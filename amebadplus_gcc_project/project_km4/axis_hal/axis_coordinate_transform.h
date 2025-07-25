#ifndef __AXIS_COORDINATE_TRANSFORM_H
#define __AXIS_COORDINATE_TRANSFORM_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include "axis_controller.h"

// === 坐标转换接口 ===

/**
 * @brief 将内部角度转换为外部角度（考虑镜像）
 * @param handle 轴控制器句柄
 * @param internal_angle 内部角度
 * @return 外部角度
 */
float axis_internal_to_external_angle(axis_handle_t *handle, float internal_angle);

/**
 * @brief 将外部角度转换为内部角度（考虑镜像）
 * @param handle 轴控制器句柄
 * @param external_angle 外部角度
 * @return 内部角度
 */
float axis_external_to_internal_angle(axis_handle_t *handle, float external_angle);

/**
 * @brief 将内部角度转换为电机步数
 * @param handle 轴控制器句柄
 * @param internal_angle 内部角度
 * @return 电机步数
 */
int axis_angle_to_motor_steps(axis_handle_t *handle, float internal_angle);

/**
 * @brief 将电机步数转换为内部角度
 * @param handle 轴控制器句柄
 * @param motor_steps 电机步数
 * @return 内部角度
 */
float axis_motor_steps_to_angle(axis_handle_t *handle, int motor_steps);

/**
 * @brief 将角度速度转换为电机PPS
 * @param handle 轴控制器句柄
 * @param deg_per_sec 角度速度(度/秒)
 * @return 电机PPS
 */
uint16_t axis_angle_velocity_to_motor_pps(axis_handle_t *handle, float deg_per_sec);

/**
 * @brief 检查内部角度是否在限位范围内
 * @param handle 轴控制器句柄
 * @param internal_angle 内部角度
 * @return true=在范围内，false=超出范围
 */
bool axis_is_internal_angle_in_limits(axis_handle_t *handle, float internal_angle);

#ifdef __cplusplus
}
#endif

#endif // __AXIS_COORDINATE_TRANSFORM_H