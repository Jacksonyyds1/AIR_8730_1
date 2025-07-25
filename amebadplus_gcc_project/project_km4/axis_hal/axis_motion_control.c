#include <math.h>
#include <stdlib.h>

#include "axis_motion_control.h"
#include "axis_coordinate_transform.h"
#include "step_motor.h" 
#include "encoder.h"

// === 辅助函数实现 ===

uint16_t axis_calculate_deceleration_steps(uint16_t current_speed, uint8_t acceleration_rate)
{
    if(current_speed <= MOTOR_MIN_PPS)
    {
        return 0;
    }
    
    // 计算从当前速度减速到最小速度需要的步数
    // 使用梯形减速公式：steps = (current_speed - min_speed) / acceleration_rate
    return (current_speed - MOTOR_MIN_PPS + acceleration_rate - 1) / acceleration_rate;
}

// === 运动控制实现 ===

bool axis_start_motor_movement(axis_handle_t *handle, int target_motor_position, uint16_t speed)
{
    if(!handle) return false;
    
    int current_position = stepper_motor_get_position(handle->config.motor_index);
    motor_direction_t direction;
    
    if(target_motor_position > current_position)
    {
        direction = Motor_Direction_Forward;
    }
    else if(target_motor_position < current_position)
    {
        direction = Motor_Direction_Backward;
    }
    else
    {
        return true; // 已经在目标位置
    }
    
    // 设置电机速度和方向
    stepper_motor_set_speed(handle->config.motor_index, speed, false);
    stepper_motor_set_acceleration_rate(handle->config.motor_index, handle->motion_params.acceleration_rate);
    stepper_motor_set_direction(handle->config.motor_index, direction, speed);
    
    return true;
}

bool axis_start_motor_velocity(axis_handle_t *handle, float velocity)
{
    if(!handle) return false;
    
    // 停止定位控制
    handle->positioning_active = false;
    
    motor_direction_t direction;
    if(velocity > 0)
    {
        direction = Motor_Direction_Forward;
    }
    else if(velocity < 0)
    {
        direction = Motor_Direction_Backward;
        velocity = -velocity; // 转为正值
    }
    else
    {
        stepper_motor_stop(handle->config.motor_index, false, false);
        return true;
    }
    
    uint16_t motor_pps = axis_angle_velocity_to_motor_pps(handle, velocity);
    stepper_motor_set_speed(handle->config.motor_index, motor_pps, false);
    stepper_motor_set_direction(handle->config.motor_index, direction, motor_pps);
    
    return true;
}

void axis_process_motion_control(axis_handle_t *handle)
{
    if(!handle) return;
    
    // 如果正在执行定位控制
    if(handle->positioning_active)
    {
        int current_position = stepper_motor_get_position(handle->config.motor_index);
        int remaining_steps = abs(handle->target_motor_position - current_position);
        
        // 获取当前速度用于计算减速距离
        uint16_t current_speed = stepper_motor_get_current_speed(handle->config.motor_index);
        
        // 计算从当前速度减速到最小速度需要的步数
        uint16_t steps_to_decelerate = stepper_motor_calc_accel_step(handle->config.motor_index);
        
        // 如果剩余步数小于等于减速步数，开始减速
        if(remaining_steps <= steps_to_decelerate + 1)
        {
            handle->positioning_active = false;
            stepper_motor_stop(handle->config.motor_index, false, false);
            LOGI("Motor %d stopping: current_position=%d, target_position=%d, remaining_steps=%d",
                 handle->config.motor_index, current_position, handle->target_motor_position, remaining_steps);
        }
    }
    else if(stepper_motor_get_state(handle->config.motor_index) == Motor_State_Stop)
    {
        // 如果电机已经停止，更新状态
        handle->state = AXIS_STATE_IDLE;
        handle->position.target_angle = handle->position.current_angle;
        LOGI("Motor %d stopped: current_angle=%.1f, motor position=%d", 
            handle->config.motor_index, handle->position.current_angle, stepper_motor_get_position(handle->config.motor_index));
    }
}

void axis_update_position_from_encoder(axis_handle_t *handle)
{
    if(!handle) return;
    
    encoder_state_t encoder_state = encoder_get_state(handle->config.motor_index);
    
    if(encoder_state == ENCODER_STATE_TRACKING)
    {
        int encoder_position = encoder_get_position(handle->config.motor_index);
        
        // 将编码器位置转换为角度
        float internal_angle = (float)encoder_position * handle->config.angle_per_encoder_unit;
        
        // 更新当前角度
        handle->position.current_angle = axis_internal_to_external_angle(handle, internal_angle);
        
        // 计算位置误差
        float error = handle->position.target_angle - handle->position.current_angle;
        handle->position.position_error = error;
    }
}