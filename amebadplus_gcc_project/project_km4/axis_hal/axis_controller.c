#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

#include "step_motor.h" 
#include "logger/logger.h"

#include "axis_coordinate_transform.h"
#include "axis_motion_control.h"
#include "axis_controller.h"
#include "encoder.h"

// 内部常量定义
#define AXIS_ANGLE_TOLERANCE_DEFAULT    0.5f    // 默认位置容差(度)
#define AXIS_POSITION_UPDATE_THRESHOLD  0.1f    // 位置更新阈值(度)
#define AXIS_POSITIONING_SPEED          5.0f    // 默认定位速度(度/秒)

// === 内部辅助函数实现 ===

static uint16_t convert_speed_to_pps(axis_handle_t *handle, float deg_per_sec)
{
    return (uint16_t)(deg_per_sec / handle->config.angle_per_encoder_unit * handle->config.motor_steps_per_unit);
}

static bool validate_config(const axis_config_t *config)
{
    if(!config)
    {
        return false;
    }

    if(config->motor_index >= 2)
    {
        return false;
    }

    if(config->motor_steps_per_unit <= 0 || config->angle_per_encoder_unit <= 0)
    {
        return false;
    }

    if(config->enable_limits && config->min_limit >= config->max_limit)
    {
        return false;
    }

    return true;
}

static void establish_position_reference(axis_handle_t *handle, bool refresh_target_angle)
{
    encoder_state_t encoder_state = encoder_get_state(handle->config.motor_index);
    
    if(encoder_state == ENCODER_STATE_TRACKING)
    {
        int encoder_position = encoder_get_position(handle->config.motor_index);
        int motor_position = stepper_motor_get_position(handle->config.motor_index);
        
        // 建立编码器位置与角度的对应关系
        float internal_angle = (float)encoder_position * handle->config.angle_per_encoder_unit;
        handle->position.current_angle = axis_internal_to_external_angle(handle, internal_angle);
        
        // 计算电机原点偏移
        handle->position.motor_origin_encoder_offset = motor_position - encoder_position;
        
        if(refresh_target_angle)
        {
            handle->position.target_angle = handle->position.current_angle;
        }
        
        // 更新运动参数
        handle->config.motor_steps_per_unit = handle->config.nominal_motor_steps_per_unit;
        handle->config.angle_per_encoder_unit = handle->config.nominal_angle_per_encoder_unit;
    }
}

// === 创建和销毁接口实现 ===

axis_handle_t *axis_create(const axis_config_t *config)
{
    if(!validate_config(config))
    {
        return NULL;
    }

    axis_handle_t *handle = (axis_handle_t *)malloc(sizeof(axis_handle_t));
    if(!handle)
    {
        return NULL;
    }

    // 初始化结构体
    memset(handle, 0, sizeof(axis_handle_t));
    
    // 拷贝配置
    memcpy(&handle->config, config, sizeof(axis_config_t));
    
    // 初始化状态
    handle->state = AXIS_STATE_UNINITIALIZED;
    handle->init_move_direction = 1; // 默认正方向
    
    // 初始化运动参数
    handle->motion_params.max_velocity = config->motion.max_velocity;
    handle->motion_params.position_tolerance = config->motion.position_tolerance;
    handle->motion_params.acceleration_rate = config->motion.acceleration_rate;
    
    // 初始化位置信息
    handle->position.min_angle_limit = config->min_limit;
    handle->position.max_angle_limit = config->max_limit;
    handle->position.limits_enabled = config->enable_limits;
    
    return handle;
}

void axis_destroy(axis_handle_t *handle)
{
    if(handle)
    {
        free(handle);
    }
}

// === 初始化接口实现 ===

bool axis_start_initialization(axis_handle_t *handle)
{
    if(!handle)
    {
        return false;
    }

    handle->state = AXIS_STATE_INIT_POSITIONING;
    handle->init_move_direction = 1; // 默认正方向
    
    return true;
}

bool axis_is_initialization_complete(axis_handle_t *handle)
{
    if(!handle)
    {
        return false;
    }

    return handle->state >= AXIS_STATE_INIT_RETURN;
}

// === 状态处理接口实现 ===

bool axis_reset(axis_handle_t *handle)
{
    if(!handle)
    {
        return false;
    }

    // 停止电机
    stepper_motor_stop(handle->config.motor_index, false, true);
    
    // 重置状态
    handle->state = AXIS_STATE_UNINITIALIZED;
    handle->positioning_active = false;
    
    return true;
}

// === 状态查询接口实现 ===

axis_state_t axis_get_state(axis_handle_t *handle)
{
    if(!handle)
    {
        return AXIS_STATE_ERROR;
    }
    
    return handle->state;
}

float axis_get_current_angle(axis_handle_t *handle)
{
    if(!handle)
    {
        return 0.0f;
    }
    
    return handle->position.current_angle;
}

float axis_get_target_angle(axis_handle_t *handle)
{
    if(!handle)
    {
        return 0.0f;
    }
    
    return handle->position.target_angle;
}

bool axis_is_in_position(axis_handle_t *handle, float target_angle)
{
    if(!handle)
    {
        return false;
    }
    
    float error = fabsf(handle->position.target_angle - target_angle);
    return error <= handle->motion_params.position_tolerance;
}

bool axis_is_stopped(axis_handle_t *handle)
{
    if(!handle)
    {
        return true;
    }
    
    return handle->state == AXIS_STATE_IDLE;
}

bool axis_get_position_info(axis_handle_t *handle, axis_position_t *position)
{
    if(!handle || !position)
    {
        return false;
    }
    
    memcpy(position, &handle->position, sizeof(axis_position_t));
    return true;
}

float axis_get_motion_progress(axis_handle_t *handle)
{
    if(!handle || handle->state != AXIS_STATE_MOVING)
    {
        return 1.0f;
    }
    
    if(handle->motion_distance == 0)
    {
        return 1.0f;
    }
    
    float current_distance = handle->position.current_angle - handle->start_angle;
    return fabsf(current_distance) / fabsf(handle->motion_distance);
}

float axis_get_rotation_range(axis_handle_t *handle)
{
    if(!handle)
    {
        return 0.0f;
    }
    
    return fabsf(handle->config.max_limit - handle->config.min_limit);
}

int axis_is_angle_speed_in_range(axis_handle_t *handle, float speed)
{
    if(!handle)
    {
        return 0;
    }
    
    if(speed <= 0)
    {
        return 0;
    }
    
    return speed <= handle->motion_params.max_velocity ? 1 : 0;
}

bool axis_is_external_angle_in_limits(axis_handle_t *handle, float angle)
{
    if(!handle || !handle->position.limits_enabled)
    {
        return true; // 如果未启用限位，则总是有效
    }
    
    return angle >= handle->position.min_angle_limit && angle <= handle->position.max_angle_limit;
}

// === 参数配置接口实现 ===

bool axis_set_motion_params(axis_handle_t *handle, const axis_motion_params_t *params)
{
    if(!handle || !params)
    {
        return false;
    }
    
    memcpy(&handle->motion_params, params, sizeof(axis_motion_params_t));
    
    // 更新电机加速率
    stepper_motor_set_acceleration_rate(handle->config.motor_index, params->acceleration_rate);
    
    return true;
}

bool axis_set_angle_limits(axis_handle_t *handle, float min_limit, float max_limit, bool enable)
{
    if(!handle)
    {
        return false;
    }
    
    if(enable && min_limit >= max_limit)
    {
        return false;
    }
    
    handle->config.min_limit = min_limit;
    handle->config.max_limit = max_limit;
    handle->config.enable_limits = enable;
    
    handle->position.min_angle_limit = min_limit;
    handle->position.max_angle_limit = max_limit;
    handle->position.limits_enabled = enable;
    
    return true;
}

// === 调试和状态字符串接口实现 ===

const char *axis_get_state_string(axis_state_t state)
{
    switch(state)
    {
    case AXIS_STATE_UNINITIALIZED:     return "UNINITIALIZED";
    case AXIS_STATE_INIT_POSITIONING:  return "INIT_POSITIONING";
    case AXIS_STATE_IDLE:              return "IDLE";
    case AXIS_STATE_MOVING:            return "MOVING";
    case AXIS_STATE_ERROR:             return "ERROR";
    default:                           return "UNKNOWN";
    }
}

const char *axis_get_result_string(axis_result_t result)
{
    switch(result)
    {
    case AXIS_RESULT_SUCCESS:         return "SUCCESS";
    case AXIS_RESULT_IN_PROGRESS:     return "IN_PROGRESS";
    case AXIS_RESULT_OUT_OF_RANGE:    return "OUT_OF_RANGE";
    case AXIS_RESULT_INVALID_PARAM:   return "INVALID_PARAM";
    case AXIS_RESULT_NOT_INITIALIZED: return "NOT_INITIALIZED";
    case AXIS_RESULT_ERROR:           return "ERROR";
    default:                          return "UNKNOWN";
    }
}

// === 运动控制接口实现 ===

axis_result_t axis_move_to_angle(axis_handle_t *handle, float target_angle, float speed)
{
    if(!handle)
    {
        return AXIS_RESULT_INVALID_PARAM;
    }

    if(handle->state <= AXIS_STATE_INIT_POSITIONING)
    {
        return AXIS_RESULT_NOT_INITIALIZED;
    }

    // 检查角度是否在限位范围内
    float internal_angle = axis_external_to_internal_angle(handle, target_angle);
    if(!axis_is_internal_angle_in_limits(handle, internal_angle))
    {
        return AXIS_RESULT_OUT_OF_RANGE;
    }

    // 计算目标电机位置
    int target_motor_steps = axis_angle_to_motor_steps(handle, internal_angle);

    // 设置目标角度
    handle->position.target_angle = target_angle;
    handle->start_angle = handle->position.current_angle;
    handle->motion_distance = target_angle - handle->start_angle;

    // 启动电机运动 - 实现定位控制逻辑
    handle->target_motor_position = target_motor_steps;
    handle->positioning_active = true;

    uint16_t speed_pps = convert_speed_to_pps(handle, speed);
    
    // 启动电机运动
    LOGI("Move motor %d to angle %.1f: target_steps=%d, speed=%d", handle->config.motor_index, target_angle, target_motor_steps, speed_pps);
    axis_start_motor_movement(handle, target_motor_steps, speed_pps);

    handle->state = AXIS_STATE_MOVING;
    return AXIS_RESULT_IN_PROGRESS;
}

axis_result_t axis_move_relative(axis_handle_t *handle, float angle_offset, float speed)
{
    if(!handle)
    {
        return AXIS_RESULT_INVALID_PARAM;
    }

    float target_angle = handle->position.current_angle + angle_offset;
    return axis_move_to_angle(handle, target_angle, speed);
}

axis_result_t axis_move_velocity(axis_handle_t *handle, float velocity_deg_per_sec)
{
    if(!handle)
    {
        return AXIS_RESULT_INVALID_PARAM;
    }

    if(handle->state <= AXIS_STATE_INIT_POSITIONING)
    {
        return AXIS_RESULT_NOT_INITIALIZED;
    }

    // 启动速度控制
    if(axis_start_motor_velocity(handle, velocity_deg_per_sec))
    {
        if(velocity_deg_per_sec == 0)
        {
            handle->state = AXIS_STATE_IDLE;
        }
        else
        {
            handle->state = AXIS_STATE_MOVING;
        }
        return AXIS_RESULT_SUCCESS;
    }
    else
    {
        return AXIS_RESULT_ERROR;
    }
}

bool axis_stop(axis_handle_t *handle, bool emergency_stop)
{
    if(!handle)
    {
        return false;
    }

    // 停止定位控制
    handle->positioning_active = false;

    stepper_motor_stop(handle->config.motor_index, false, emergency_stop);

    if(emergency_stop)
    {
        handle->state = AXIS_STATE_IDLE;
    }
    
    return true;
}

// === 更新和处理接口实现 ===

static void process_init_positioning(axis_handle_t *handle)
{
    if(stepper_motor_get_state(handle->config.motor_index) == Motor_State_Stop)
    {
        motor_direction_t direction = handle->init_move_direction > 0 ? Motor_Direction_Forward : Motor_Direction_Backward;
        uint16_t target_pps = convert_speed_to_pps(handle, AXIS_POSITIONING_SPEED);
        stepper_motor_set_direction(handle->config.motor_index, direction, target_pps);
    }
    
    encoder_state_t encoder_state = encoder_get_state(handle->config.motor_index);
    
    if(encoder_state == ENCODER_STATE_TRACKING)
    {
        LOGI("encoder tracking established, position: %d", encoder_get_position(handle->config.motor_index));
        establish_position_reference(handle, true);
        handle->positioning_active = false;
        handle->state = AXIS_STATE_INIT_RETURN;
    }
    else
    {
        handle->state = AXIS_STATE_INIT_POSITIONING;
    }
}

static void process_init_return(axis_handle_t *handle)
{
    float zero_angle = axis_motor_steps_to_angle(handle, 0);
    zero_angle = axis_internal_to_external_angle(handle, zero_angle);
    if(axis_is_internal_angle_in_limits(handle, zero_angle))
    {
        axis_move_to_angle(handle, zero_angle, AXIS_POSITIONING_SPEED);
    }
    else if(zero_angle < handle->config.min_limit)
    {
        axis_move_to_angle(handle, handle->config.min_limit, AXIS_POSITIONING_SPEED);
    }
    else if(zero_angle > handle->config.max_limit)
    {
        axis_move_to_angle(handle, handle->config.max_limit, AXIS_POSITIONING_SPEED);
    }
}

static void process_moving(axis_handle_t *handle)
{
    if(!handle)
    {
        return;
    }
    
    motor_state_t motor_state = stepper_motor_get_state(handle->config.motor_index);

    if(motor_state == Motor_State_Stop)
    {
        handle->state = AXIS_STATE_IDLE;
        handle->position.target_angle = handle->position.current_angle;
        handle->positioning_active = false;
        return;
    }

    axis_process_motion_control(handle);
}

void axis_update(axis_handle_t *handle)
{
    if(!handle)
    {
        return;
    }

    // 处理状态机
    switch(handle->state)
    {
    case AXIS_STATE_UNINITIALIZED:
        // 在未初始化状态，等待用户调用axis_start_initialization
        break;
    
    case AXIS_STATE_INIT_POSITIONING:
        process_init_positioning(handle);
        break;

    case AXIS_STATE_INIT_RETURN:
        process_init_return(handle);
        LOGI("Axis %d: returning to initial position", handle->config.motor_index);
        break;

    case AXIS_STATE_IDLE:
        break;
        
    case AXIS_STATE_MOVING:
        axis_process_motion_control(handle);
        break;
        
    default:
        break;
    }
}