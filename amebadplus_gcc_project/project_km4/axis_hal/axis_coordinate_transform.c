#include "axis_coordinate_transform.h"
#include "stepper_motor/stepper_motor.h"
#include <math.h>

// === 内部辅助函数 ===

// 由于所有函数都需要访问axis_controller_s结构，我们需要在这里声明
// 这个结构体定义在axis_controller.c中，通过axis_handle_t访问

// === 坐标转换实现 ===

float axis_external_to_internal_angle(axis_handle_t *handle, float external_angle)
{
    if(!handle) return 0.0f;
    
    // 获取配置 - 这里需要访问handle->config
    // 由于结构体是不透明的，我们需要通过接口获取配置信息
    // 暂时假设我们可以直接访问，后续可能需要调整
    
    // 如果启用了位置镜像
    if(handle->config.mirror_position)
    {
        // 计算限位范围的中点
        float mid_point = (handle->config.min_limit + handle->config.max_limit) / 2.0f;
        // 以中点为轴进行镜像
        return 2.0f * mid_point - external_angle;
    }
    return external_angle;
}

float axis_internal_to_external_angle(axis_handle_t *handle, float internal_angle)
{
    if(!handle) return 0.0f;
    
    if(handle->config.mirror_position)
    {
        // 计算限位范围的中点
        float mid_point = (handle->config.min_limit + handle->config.max_limit) / 2.0f;
        // 以中点为轴进行镜像
        return 2.0f * mid_point - internal_angle;
    }
    return internal_angle;
}

int axis_internal_angle_to_motor_steps(axis_handle_t *handle, float internal_angle)
{
    if(!handle) return 0;
    
    // 将角度转换为编码器单位
    float encoder_units = internal_angle / handle->config.angle_per_encoder_unit;
    
    // 再转换为电机步数
    return (int)(encoder_units * handle->config.motor_steps_per_unit);
}

float axis_motor_steps_to_angle(axis_handle_t *handle, int motor_steps)
{
    if(!handle) return 0.0f;
    
    // 将电机步数转换为编码器单位
    float encoder_units = (float)motor_steps / handle->config.motor_steps_per_unit;
    
    // 再转换为内部角度
    return encoder_units * handle->config.angle_per_encoder_unit;
}

uint16_t axis_angle_velocity_to_motor_pps(axis_handle_t *handle, float deg_per_sec)
{
    if(!handle || deg_per_sec <= 0) return MOTOR_MIN_PPS;
    
    // 将角度速度转换为电机PPS
    float encoder_units_per_sec = deg_per_sec / handle->config.angle_per_encoder_unit;
    float motor_pps = encoder_units_per_sec * handle->config.motor_steps_per_unit;
    
    // 限制在合理范围内
    if(motor_pps < MOTOR_MIN_PPS) return MOTOR_MIN_PPS;
    if(motor_pps > 60000) return 60000; // 最大频率限制
    
    return (uint16_t)motor_pps;
}

bool axis_is_angle_in_limits(axis_handle_t *handle, float angle)
{
    if(!handle) return false;
    
    if(!handle->config.enable_limits) return true;
    
    // 将外部限位转换为内部限位进行比较
    float min_limit, max_limit;
    
    // 确保min < max
    if(handle->config.min_limit > handle->config.max_limit)
    {
        min_limit = handle->config.max_limit;
        max_limit = handle->config.min_limit;
    }
    else
    {
        min_limit = handle->config.min_limit;
        max_limit = handle->config.max_limit;
    }

    return (angle >= min_limit && angle <= max_limit);
}