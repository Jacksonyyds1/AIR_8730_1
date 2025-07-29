// RTL8721DCM系统头文件
#include "ameba_soc.h"
#include "peripheral_wrapper.h"
// 数学库
#include "arm_math.h"
#include <cmath>

// 标准库
#include <algorithm>
#include <cstring>

#include "step_motor.h"
#include "hinge_joint.hpp"

#include <stdint.h>
#include <stdbool.h>

#include "FreeRTOS.h"
#include "task.h"
#include "logger/logger.h"


#include "posture_controller.h"


#define sys_tick() xTaskGetTickCount()

PostureController::HingeJoint &axis_system = PostureController::HingeJoint::getInstance();
static axis_handle_t *base_axis, *neck_axis;

typedef struct {
    posture_controller_state_t state;       // 当前状态

    oscillation_state_t base_oscillation_state;    // 基座扫描状态
    uint32_t base_oscillation_reverse_tick;        // 基座扫描反向计时器
    float32_t base_oscillation_range_deg;          // 基座扫描范围

    oscillation_state_t neck_oscillation_state;    // 颈部扫描状态
    uint32_t neck_oscillation_reverse_tick;        // 颈部扫描反向计时器
    float32_t neck_oscillation_range_deg;          // 颈部扫描范围

    float32_t base_angle_target_deg;          // 基座目标角度

    float32_t nozzle_elevation_target_deg;    // 喷嘴仰角目标
    float32_t base_angle_initial_deg;         // 基座初始角度
    float32_t neck_angle_rotation_deg;        // 颈部目标角度
} posture_runtime_t;

posture_runtime_t posture_runtime = {
    .state = Posture_Controller_State_Uninitialized,
    .base_oscillation_range_deg = 0.0f,
    .base_angle_target_deg = 0.0f,
    .nozzle_elevation_target_deg = 0.0f

};

/**
 * @brief Initialize the posture controller
 * @param None
 */
static void posture_controller_init(void)
{
    // 喷嘴轴配置
    axis_config_t neck_config =
    {
        .motor_index = MOTOR_NECK,
        .enable_limits = true,
        .mirror_position = true,
        .mirror_motion = false,
        .nominal_motor_steps_per_unit = NECK_MOTOR_STEPS_PER_DEGREE / ENCODER_UNITS_PER_DEG,
        .motor_steps_per_unit = NECK_MOTOR_STEPS_PER_DEGREE / ENCODER_UNITS_PER_DEG,
        .min_limit = 0.0f,
        .max_limit = 180.0f,
        .nominal_angle_per_encoder_unit = 1.0f,
        .angle_per_encoder_unit = 1.0f,
        .motion = {
            .max_velocity = 18.0f,
            .position_tolerance = 1.0f,
            .acceleration_rate = 14,
        },
    };

    // 底座轴配置
    axis_config_t base_config =
    {
        .motor_index = MOTOR_BASE,
        .enable_limits = true,
        .mirror_position = true,
        .mirror_motion = false,
        .nominal_motor_steps_per_unit = BASE_MOTOR_STEPS_PER_DEGREE / ENCODER_UNITS_PER_DEG,
        .motor_steps_per_unit = BASE_MOTOR_STEPS_PER_DEGREE / ENCODER_UNITS_PER_DEG,
        .min_limit = 0.0f,
        .max_limit = 350.0f,
        .nominal_angle_per_encoder_unit = 1.0f,
        .angle_per_encoder_unit = 1.0f,
        .motion = {
            .max_velocity = 12.0f,
            .position_tolerance = 1.0f,
            .acceleration_rate = 12,
        },
    };

    stepper_motor_init();
    encoder_init();
    encoder_enable_simulated_signals(true); // Enable simulated encoder signals for testing

    // 初始化轴控制器
    base_axis = axis_create(&base_config);
    neck_axis = axis_create(&neck_config);

    axis_system.init_axis(base_axis, neck_axis);
    axis_system.init_posture();

    axis_start_initialization(base_axis);
    axis_start_initialization(neck_axis);
}

/**
 * @brief Process the position control state
 * @param None
 */
static void position_control_state_process(void)
{
    if(axis_is_stopped(base_axis))
    {
        if(axis_is_in_position(base_axis, posture_runtime.base_angle_target_deg, 1.0f))
        {
            posture_runtime.state = Posture_Controller_State_Idle;
            LOGI("Base axis reached target position: %.2f degrees", base_axis->position.current_angle);
        }
        else
        {
            // 重新设置目标角度
            axis_move_to_angle(base_axis, posture_runtime.base_angle_target_deg, base_axis->config.motion.max_velocity);

            LOGW("Base axis is not in position: %.2f degrees", posture_runtime.base_angle_target_deg);
            LOGW("Current angle: %.2f degrees, error: %.2f degrees",
                 base_axis->position.current_angle,
                 posture_runtime.base_angle_target_deg - base_axis->position.current_angle);
        }
    }
}

/**
 * @brief Process the scan motion state
 * @param None
 */
static void base_oscillation_state_process(void)
{
    bool set_target = false;
    float center_angle = (base_axis->config.min_limit + base_axis->config.max_limit) / 2.0f;
    float start_angle = center_angle - posture_runtime.base_oscillation_range_deg / 2.0f;
    float end_angle = center_angle + posture_runtime.base_oscillation_range_deg / 2.0f;

    switch(posture_runtime.base_oscillation_state)
    {
    case Oscillation_State_Start:
        set_target = true;
        if(axis_is_stopped(base_axis) || axis_get_target_velocity(base_axis) == 0)
        {
            if(axis_get_current_angle(base_axis) < center_angle)
            {
                posture_runtime.base_oscillation_state = Oscillation_State_Forward;
                LOGI("Base scan state: Forward");
            }
            else
            {
                posture_runtime.base_oscillation_state = Oscillation_State_Backward;
                LOGI("Base scan state: Backward");
            }
        }
        else
        {
            if(axis_get_current_angle(base_axis) <= start_angle || axis_get_target_velocity(base_axis) < 0)
            {
                posture_runtime.base_oscillation_state = Oscillation_State_Forward;
                LOGI("Base scan state: Forward");
            }
            else if(axis_get_current_angle(base_axis) >= end_angle || axis_get_target_velocity(base_axis) > 0)
            {
                posture_runtime.base_oscillation_state = Oscillation_State_Backward;
                LOGI("Base scan state: Backward");
            }
        }
        break;

    case Oscillation_State_Forward:
        if(axis_is_stopped(base_axis))
        {
            if(axis_is_in_position(base_axis, end_angle, 1.0f))
            {
                posture_runtime.base_oscillation_reverse_tick = sys_tick() + 3000;
                posture_runtime.base_oscillation_state = Oscillation_State_Wait;
                LOGI("Base scan state: Wait for reverse");
            }
            else
            {
                set_target = true;
            }
        }
        break;

    case Oscillation_State_Backward:
        if(axis_is_stopped(base_axis))
        {
            if(axis_is_in_position(base_axis, start_angle, 1.0f))
            {
                posture_runtime.base_oscillation_reverse_tick = sys_tick() + 3000;
                posture_runtime.base_oscillation_state = Oscillation_State_Wait;
                LOGI("Base scan state: Wait for reverse");
            }
            else
            {
                set_target = true;
            }
        }
        break;

    case Oscillation_State_Wait:
        if(sys_tick() >= posture_runtime.base_oscillation_reverse_tick)
        {
            posture_runtime.base_oscillation_state = Oscillation_State_Start;
        }
        break;
    }

    if(set_target)
    {
        if(posture_runtime.base_oscillation_state == Oscillation_State_Forward)
        {
            axis_move_to_angle(base_axis, end_angle, base_axis->config.motion.max_velocity);
        }
        else if(posture_runtime.base_oscillation_state == Oscillation_State_Backward)
        {
            axis_move_to_angle(base_axis, start_angle, base_axis->config.motion.max_velocity);
        }
    }
}

/**
 * @brief Process the scan motion state
 * @param None
 */
static void neck_oscillation_state_process(void)
{
    bool set_target = false;
    float center_angle = (neck_axis->config.min_limit + neck_axis->config.max_limit) / 2.0f;
    float start_angle = center_angle - posture_runtime.neck_oscillation_range_deg / 2.0f;
    float end_angle = center_angle + posture_runtime.neck_oscillation_range_deg / 2.0f;

    switch(posture_runtime.neck_oscillation_state)
    {
    case Oscillation_State_Start:
        set_target = true;
        if(axis_is_stopped(neck_axis) || axis_get_target_velocity(neck_axis) == 0)
        {
            if(axis_get_current_angle(neck_axis) < center_angle)
            {
                posture_runtime.neck_oscillation_state = Oscillation_State_Forward;
                LOGI("Neck scan state: Forward");
            }
            else
            {
                posture_runtime.neck_oscillation_state = Oscillation_State_Backward;
                LOGI("Neck scan state: Backward");
            }
        }
        else
        {
            if(axis_get_current_angle(neck_axis) <= start_angle || axis_get_target_velocity(neck_axis) < 0)
            {
                posture_runtime.neck_oscillation_state = Oscillation_State_Forward;
                LOGI("Neck scan state: Forward");
            }
            else if(axis_get_current_angle(neck_axis) >= end_angle || axis_get_target_velocity(neck_axis) > 0)
            {
                posture_runtime.neck_oscillation_state = Oscillation_State_Backward;
                LOGI("Neck scan state: Backward");
            }
        }
        break;

    case Oscillation_State_Forward:
        if(axis_is_stopped(neck_axis))
        {
            if(axis_is_in_position(neck_axis, end_angle, 1.0f))
            {
                posture_runtime.neck_oscillation_reverse_tick = sys_tick() + 3000;
                posture_runtime.neck_oscillation_state = Oscillation_State_Wait;
                LOGI("Neck scan state: Wait for reverse");
            }
            else
            {
                set_target = true;
            }
        }
        break;

    case Oscillation_State_Backward:
        if(axis_is_stopped(neck_axis))
        {
            if(axis_is_in_position(neck_axis, start_angle, 1.0f))
            {
                posture_runtime.neck_oscillation_reverse_tick = sys_tick() + 3000;
                posture_runtime.neck_oscillation_state = Oscillation_State_Wait;
                LOGI("Neck scan state: Wait for reverse");
            }
            else
            {
                set_target = true;
            }
        }
        break;

    case Oscillation_State_Wait:
        if(sys_tick() >= posture_runtime.neck_oscillation_reverse_tick)
        {
            posture_runtime.neck_oscillation_state = Oscillation_State_Start;
        }
        break;
    }

    if(set_target)
    {
        if(posture_runtime.neck_oscillation_state == Oscillation_State_Forward)
        {
            axis_move_to_angle(neck_axis, end_angle, neck_axis->config.motion.max_velocity);
        }
        else if(posture_runtime.neck_oscillation_state == Oscillation_State_Backward)
        {
            axis_move_to_angle(neck_axis, start_angle, neck_axis->config.motion.max_velocity);
        }
    }
}

static void elevation_control_state_process(void)
{
    const SphericalCoord current_coord = axis_system.get_nozzle_spherical_coord();
    SphericalCoord target_coord(current_coord.azimuth, posture_runtime.nozzle_elevation_target_deg);
    Vector3 current_vector, target_vector;
    current_vector = current_coord.toVector();
    target_vector = target_coord.toVector();
    
    Quaternion rotate_axis = Quaternion::fromVectors(current_vector, target_vector);
    

    if(axis_is_in_position(neck_axis, posture_runtime.neck_angle_rotation_deg, 0.1f))
    {
        // 停止颈部运动，并将base轴旋转回到初始角度
        axis_stop(neck_axis, false);
        axis_move_to_angle(base_axis, posture_runtime.base_angle_initial_deg, base_axis->config.motion.max_velocity);
        posture_runtime.state = Posture_Controller_State_Idle;

        LOGI("Neck axis arrived target elevation: %.2f degrees", posture_runtime.nozzle_elevation_target_deg);
        LOGI("Neck axis position: %.2f degrees", axis_get_current_angle(neck_axis));
        LOGI("Base axis position: %.2f degrees", axis_get_current_angle(base_axis));
        return; // 已经在目标位置
    }

    // 获取当前角度
    //float current_base_deg = axis_system.get_base_angle_deg();
    float current_neck_deg = axis_system.get_neck_angle_deg();
    
    // 确定neck轴旋转方向和速度
    float neck_deg_error = posture_runtime.neck_angle_rotation_deg - current_neck_deg;
   // bool neck_counterclockwise = (posture_runtime.nozzle_elevation_target_deg > current_coord.elevation); // 目标仰角高于当前：逆时针

    // neck轴使用位置控制，以最大速度移动
    bool neck_success = false;
    
    if(neck_axis && fabsf(neck_deg_error) > 0.1f) {
        // 使用位置控制移动到目标角度
        if(axis_move_to_angle(neck_axis, posture_runtime.neck_angle_rotation_deg, neck_axis->config.motion.max_velocity) == AXIS_RESULT_SUCCESS) {
            neck_success = true;
            LOGD("Neck axis rotate: current=%.1f°, 目标=%.1f°, 误差=%.1f°\r\n",
                 current_neck_deg, posture_runtime.neck_angle_rotation_deg, neck_deg_error);
        }
    } else {
        neck_success = true; // 角度误差很小，认为已到位
    }
    
    // 计算neck轴的实际角速度（估算）
    float neck_velocity = axis_get_current_velocity(neck_axis);
    
    // 计算nozzle在xy平面投影的角速度补偿
    float base_compensation_velocity = axis_system.calculate_base_compensation_velocity(neck_velocity);
    
    // base轴使用速度控制进行补偿
    axis_move_velocity(base_axis, base_compensation_velocity, true);
}

float slove_neck_angle_for_nozzle_elevation(float target_elevation_deg)
{
    return axis_system.slove_neck_angle_for_nozzle_elevation(target_elevation_deg);
}

float calculate_base_compensation_velocity(float neck_velocity_dps)
{
    return axis_system.calculate_base_compensation_velocity(neck_velocity_dps);
}

/**
 * @brief Process the posture controller state machine
 * @param None
 */
void posture_controller_state_process(void)
{
    if(posture_runtime.state >= Posture_Controller_State_Initializing)
    {
        axis_system.update_axis();
        axis_system.update_posture();
    }

    switch(posture_runtime.state)
    {
    case Posture_Controller_State_Uninitialized:
        posture_runtime.state = Posture_Controller_State_Initializing;
        posture_controller_init();
        LOGI("Posture controller state: Initializing");
        break;

    case Posture_Controller_State_Initializing:
        if(axis_system.is_initialization_complete())
        {
            posture_runtime.state = Posture_Controller_State_Idle;
            LOGI("Posture controller state: Idle");
        }
        break;

    case Posture_Controller_State_Idle:
        break;

    case Posture_Controller_State_Position_Control:  // 位置控制模式
        position_control_state_process();
        break;

    case Posture_Controller_State_Base_Oscillation:  // 摆头运动模式
        base_oscillation_state_process();
        break;
        
    case Posture_Controller_State_Elevation_Control: // 仰角控制模式
        elevation_control_state_process();
        break;

    case Posture_Controller_State_Test_Mode:         // 测试模式
        base_oscillation_state_process();
        neck_oscillation_state_process();
        break;

    default:
        posture_runtime.state = Posture_Controller_State_Error;
        LOGE("Posture controller state: Error");
        break;
    }
}

/**
 * @brief Task to handle the posture controller state machine
 * @param pvParameters Pointer to task parameters (not used)
 */
void axis_system_task(void *pvParameters)
{
    UNUSED(pvParameters);
    TickType_t xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount(); // Initialize the last wake time

    while(1)
    {
        posture_controller_state_process();
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1)); // 10ms delay for motor update
    }
}

extern "C" {
    
/**
 * @brief Initialize the posture controller task
 * @param None
 */
void posture_controller_task_init(void)
{
    // 启动任务
    if(xTaskCreate(axis_system_task, "PoseCtrlTask", 4096, NULL, 4, NULL) == pdPASS)
    {
        LOGI("Posture controller initialized successfully.");
    }
    else
    {
        LOGE("Failed to create posture controller task.");
    }
}

/**
 * @brief Check if the posture controller is ready
 * @param None
 * @return true if ready, false otherwise
 */
bool posture_controller_is_ready(void)
{
    return posture_runtime.state > Posture_Controller_State_Initializing;
}

/**
 * @brief Move the base axis to a specified angle
 * @param angle_deg Target angle in degrees
 * @return true if the operation was successful, false otherwise
 */
bool posture_controller_move_base(float angle_deg)
{
    if(!posture_controller_is_ready())
    {
        LOGE("Posture control failed, controller not initialized.");
        return false;
    }

    if(!axis_is_external_angle_in_limits(base_axis, angle_deg))
    {
        LOGE("Target angle %.2f degrees out of range [%f, %f]", angle_deg,
             base_axis->config.min_limit, base_axis->config.max_limit);
        return false;
    }

    // 设置基座目标角度并开始位置控制
    axis_stop(neck_axis, false); // 停止喷嘴轴运动
    axis_move_to_angle(base_axis, angle_deg, base_axis->config.motion.max_velocity);
    posture_runtime.base_angle_target_deg = angle_deg;
    posture_runtime.state = Posture_Controller_State_Position_Control;
    LOGI("Posture controller moving base to angle: %.2f degrees", angle_deg);

    return true;
}

/**
 * @brief Set the base scan range for the posture controller
 * @param range_deg Scan range in degrees (e.g., 350.0f for full rotation)
 * @return true if the operation was successful, false otherwise
 */
bool posture_controller_set_base_oscillation_range(float range_deg)
{
    if(posture_runtime.state <= Posture_Controller_State_Initializing)
    {
        LOGE("Posture control failed, controller not initialized.");
        return false;
    }

    if(range_deg < 0.0f || range_deg > axis_get_rotation_range(base_axis))
    {
        LOGE("Base scan range %.2f degrees out of range [0.0, %.2f]", range_deg, axis_get_rotation_range(base_axis));
        return false;
    }

    // 设置基座扫描范围
    axis_stop(neck_axis, false); // 停止喷嘴轴运动
    posture_runtime.base_oscillation_range_deg = range_deg;
    posture_runtime.base_oscillation_state = Oscillation_State_Start;
    posture_runtime.state = Posture_Controller_State_Base_Oscillation;

    LOGI("Posture controller setting base scan range to: %.2f degrees", range_deg);
    return true;
}

/**
 * @brief Set the nozzle elevation angle for the posture controller
 * @param angle_deg Target elevation angle in degrees
 * @return true if the operation was successful, false otherwise
 */
bool posture_controller_set_elevation(float angle_deg)
{
    if(posture_runtime.state <= Posture_Controller_State_Initializing)
    {
        LOGE("Posture control failed, controller not initialized.");
        return false;
    }

    const SphericalCoord current_coord = axis_system.get_nozzle_spherical_coord();
    SphericalCoord target_coord(current_coord.azimuth, posture_runtime.nozzle_elevation_target_deg);
    Vector3 target_vector = target_coord.toVector();

    // 设置喷嘴仰角目标
    posture_runtime.nozzle_elevation_target_deg = angle_deg;
    posture_runtime.base_angle_initial_deg = axis_get_current_angle(base_axis);
    posture_runtime.neck_angle_rotation_deg = axis_system.slove_neck_angle_for_nozzle_elevation(angle_deg);
    posture_runtime.state = Posture_Controller_State_Elevation_Control;
    LOGI("Posture controller setting nozzle elevation to: azimuth=%.2f, elevation=%.2f",
         target_coord.azimuth, target_coord.elevation);
    LOGI("Aka vector: x=%.2f, y=%.2f, z=%.2f", target_vector.x, target_vector.y, target_vector.z);
    return true;
}

/**
 * @brief Set the posture controller to test mode
 * @param None
 * @return true if the operation was successful, false otherwise
 */
bool posture_controller_set_test_mode(void)
{
    if(posture_runtime.state <= Posture_Controller_State_Initializing)
    {
        LOGE("Posture control failed, controller not initialized.");
        return false;
    }

    posture_runtime.base_oscillation_range_deg = 340.0f; // 设置基座扫描范围为340度
    posture_runtime.base_oscillation_state = Oscillation_State_Start;
    posture_runtime.neck_oscillation_range_deg = 180.0f; // 设置颈部扫描范围为180度
    posture_runtime.neck_oscillation_state = Oscillation_State_Start;
    
    posture_runtime.state = Posture_Controller_State_Test_Mode;
    LOGI("Posture controller set to test mode.");
    return true;
}

/**
 * @brief Stop the posture controller
 * @param None
 * @return true if successful, false otherwise
 */
bool posture_controller_stop(void)
{
    if(posture_runtime.state <= Posture_Controller_State_Initializing)
    {
        LOGE("Posture control failed, controller not initialized.");
        return false;
    }

    // 停止所有运动
    axis_stop(base_axis, false);
    axis_stop(neck_axis, false);

    posture_runtime.state = Posture_Controller_State_Idle;
    posture_runtime.base_oscillation_state = Oscillation_State_Start;
    posture_runtime.base_oscillation_reverse_tick = 0;
    posture_runtime.base_oscillation_range_deg = 0.0f;
    posture_runtime.base_angle_target_deg = 0.0f;
    posture_runtime.nozzle_elevation_target_deg = 0.0f;

    LOGI("Posture controller stopped and reset to idle state.");
    return true;
}

}
