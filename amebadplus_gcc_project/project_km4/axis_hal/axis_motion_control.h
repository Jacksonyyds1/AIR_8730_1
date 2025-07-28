#ifndef __AXIS_MOTION_CONTROL_H
#define __AXIS_MOTION_CONTROL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include "axis_controller.h"

// === 运动控制接口 ===

/**
 * @brief 计算减速所需的步数
 * @param current_speed 当前速度(PPS)
 * @param acceleration_rate 加速率
 * @return 减速所需步数
 */
uint16_t axis_calculate_deceleration_steps(uint16_t current_speed, uint8_t acceleration_rate);

/**
 * @brief 启动电机定位运动
 * @param handle 轴控制器句柄
 * @param target_motor_position 目标电机位置(步数)
 * @param speed 运动速度(PPS)
 * @return 成功返回true
 */
bool axis_start_motor_movement(axis_handle_t *handle, int target_motor_position, uint16_t speed);

/**
 * @brief 启动电机速度控制运动
 * @param handle 轴控制器句柄
 * @param velocity 速度(度/秒)，正数向前，负数向后
 * @param immediate 是否跳过加速过程，立即开始运动
 * @return 成功返回true
 */
bool axis_start_motor_velocity(axis_handle_t *handle, float velocity, bool immediate);

/**
 * @brief 处理运动控制逻辑
 * @param handle 轴控制器句柄
 */
void axis_process_motion_control(axis_handle_t *handle);

/**
 * @brief 从编码器更新位置信息
 * @param handle 轴控制器句柄
 */
void axis_update_position_from_encoder(axis_handle_t *handle);

#ifdef __cplusplus
}
#endif

#endif // __AXIS_MOTION_CONTROL_H