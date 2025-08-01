#ifndef __AXIS_CONTROLLER_H
#define __AXIS_CONTROLLER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#include "encoder.h"

// 轴控制器状态枚举
typedef enum {
    AXIS_STATE_UNINITIALIZED,        // 未初始化
    AXIS_STATE_INIT_POSITIONING,     // 慢速移动寻找参考点
    AXIS_STATE_INIT_RETURN,          // 返回初始位置
    AXIS_STATE_IDLE,                 // 空闲
    AXIS_STATE_MOVING,               // 运动中
    AXIS_STATE_ERROR,                // 错误状态
} axis_state_t;

// 轴运动结果
typedef enum {
    AXIS_RESULT_SUCCESS,         // 成功
    AXIS_RESULT_IN_PROGRESS,     // 进行中
    AXIS_RESULT_OUT_OF_RANGE,    // 超出限位范围
    AXIS_RESULT_INVALID_PARAM,   // 无效参数
    AXIS_RESULT_NOT_INITIALIZED, // 未初始化
    AXIS_RESULT_ERROR            // 错误
} axis_result_t;

// 轴位置信息
typedef struct {
    float current_angle;                // 当前角度位置(度)
    float target_angle;                 // 目标角度位置(度)
    float min_angle_limit;              // 最小角度限位
    float max_angle_limit;              // 最大角度限位
    int motor_origin_encoder_offset;    // 电机原点偏移量(编码器单位)
    bool limits_enabled;                // 限位是否启用
    float position_error;               // 位置误差(度)
} axis_position_t;

// 轴运动参数
typedef struct {
    float max_velocity;           // 最大速度(度/秒)
    float position_tolerance;     // 位置容差(度)
    uint8_t acceleration_rate;    // 电机加速率
} axis_motion_params_t;

// 轴控制器配置
typedef struct {
    uint8_t motor_index;                 // 电机索引 (MOTOR_NECK 或 MOTOR_BASE)
    bool enable_limits;                  // 是否启用限位
    bool mirror_position;                // 位置镜像：true=镜像外部接口的位置输入输出，内部计算使用原始数据
    bool mirror_motion;                  // 运动镜像：true=镜像电机运动方向和码盘正方向的关系
    float nominal_motor_steps_per_unit;  // 电机每个编码器单位对应的步数（标称值）
    float motor_steps_per_unit;          // 电机每个编码器单位对应的步数，运动时实时校准
    float min_limit, max_limit;          // 角度限位(度) - 这里是外部接口的角度范围
    float nominal_angle_per_encoder_unit;// 每个编码器单位对应的角度(通常为1.0)
    float angle_per_encoder_unit;        // 每个编码器单位对应的角度(通常为1.0)
    axis_motion_params_t motion;         // 运动参数
} axis_config_t;

// 轴控制器内部结构
typedef struct
{
    // 配置参数
    axis_config_t config;

    // 当前状态
    axis_state_t state;
    axis_position_t position;
    axis_motion_params_t motion_params;
    encoder_state_t encoder_state;      // 编码器状态
    encoder_state_t last_encoder_state; // 上一个编码器状态
    
    // 控制相关
    float target_velocity;           // 运动速度
    int target_motor_position;       // 目标电机位置（步数）
    bool positioning_active;         // 前往目标位置中
    
    // 初始化相关
    int init_move_direction;

    // 统计和调试
    uint32_t last_update_time;
    uint32_t error_count;
} axis_handle_t;

// === 辅助函数声明 ===

/**
 * @brief 将角速度转换为电机脉冲速度
 * @param handle 轴控制器句柄
 * @param deg_per_sec 角速度(度/秒)
 * @return 电机脉冲速度(每秒脉冲数)
 */
uint16_t convert_speed_to_pps(axis_handle_t *handle, float deg_per_sec);

/**
 * @brief 将电机脉冲速度转换为角速度
 * @param handle 轴控制器句柄
 * @param pps 电机脉冲速度
 * @return 角速度(度/秒)
 */
float convert_pps_to_speed(axis_handle_t *handle, uint16_t pps);

// === 创建和销毁接口 ===

/**
 * @brief 创建轴控制器
 * @param config 轴控制器配置
 * @return 轴控制器句柄，失败返回NULL
 */
axis_handle_t *axis_create(const axis_config_t *config);

/**
 * @brief 销毁轴控制器
 * @param handle 轴控制器句柄
 */
void axis_destroy(axis_handle_t *handle);

// === 初始化接口 ===

/**
 * @brief 启动轴初始化
 * @param handle 轴控制器句柄
 * @return 是否成功启动
 */
bool axis_start_initialization(axis_handle_t *handle);

/**
 * @brief 检查初始化是否完成
 * @param handle 轴控制器句柄
 * @return 是否初始化完成
 */
bool axis_is_initialization_complete(axis_handle_t *handle);

// === 运动控制接口 ===

/**
 * @brief 移动到绝对角度位置
 * @param handle 轴控制器句柄
 * @param target_angle 目标角度(度)
 * @param speed 运动速度(度/秒)
 * @return 运动结果
 */
axis_result_t axis_move_to_angle(axis_handle_t *handle, float target_angle, float speed);

/**
 * @brief 相对位置移动
 * @param handle 轴控制器句柄
 * @param angle_offset 角度偏移量(度)
 * @param speed 运动速度(度/秒)
 * @return 运动结果
 */
axis_result_t axis_move_relative(axis_handle_t *handle, float angle_offset, float speed);

/**
 * @brief 以指定速度连续移动
 * @param handle 轴控制器句柄
 * @param velocity_deg_per_sec 速度(度/秒)，正数向前，负数向后
 * @param immediate 是否跳过加速过程，立即开始运动
 * @return 运动结果
 */
axis_result_t axis_move_velocity(axis_handle_t *handle, float velocity_deg_per_sec, bool immediate);

/**
 * @brief 停止运动
 * @param handle 轴控制器句柄
 * @param emergency_stop 是否紧急停止
 * @return 是否成功
 */
bool axis_stop(axis_handle_t *handle, bool emergency_stop);

// === 状态查询接口 ===

/**
 * @brief 获取轴当前状态
 * @param handle 轴控制器句柄
 * @return 轴状态
 */
axis_state_t axis_get_state(axis_handle_t *handle);

/**
 * @brief 获取当前角度
 * @param handle 轴控制器句柄
 * @return 当前角度(度)
 */
float axis_get_current_angle(axis_handle_t *handle);

/**
 * @brief 获取目标角度
 * @param handle 轴控制器句柄
 * @return 目标角度(度)
 */
float axis_get_target_angle(axis_handle_t *handle);

/**
 * @brief 获取当前速度
 * @param handle 轴控制器句柄
 * @return 当前速度(度/秒)
 */
float axis_get_current_velocity(axis_handle_t *handle);

/**
 * @brief 获取目标速度
 * @param handle 轴控制器句柄
 * @return 目标速度(度/秒)
 */
float axis_get_target_velocity(axis_handle_t *handle);

/**
 * @brief 检查是否位于目标位置
 * @param handle 轴控制器句柄
 * @param target_angle 目标角度(度)
 * @return 是否到位
 */
bool axis_is_in_position(axis_handle_t *handle, float target_angle, float tolerance);

/**
 * @brief 检查是否到达目标位置
 * @param handle 轴控制器句柄
 * @return 是否到位
 */
bool axis_is_arrived(axis_handle_t *handle);

/**
 * @brief 检查轴是否停止
 * @param handle 轴控制器句柄
 * @return 是否停止
 */
bool axis_is_stopped(axis_handle_t *handle);

/**
 * @brief 获取位置信息
 * @param handle 轴控制器句柄
 * @param position 输出位置信息
 * @return 是否成功
 */
bool axis_get_position_info(axis_handle_t *handle, axis_position_t *position);

/**
 * @brief 获取旋转范围
 * @param handle 轴控制器句柄
 * @return 旋转范围(度)
 */
float axis_get_rotation_range(axis_handle_t *handle);

/**
 * @brief 检查角速度是否在允许范围内
 * @param handle 轴控制器句柄
 * @param speed 角速度(度/秒)
 * @return 0=在范围内, 1=超出上限, -1=低于下限
 */
int axis_is_angle_speed_in_range(axis_handle_t *handle, float speed);

/**
 * @brief 检查外部角度是否在限位范围内
 * @param handle 轴控制器句柄
 * @param angle 外部角度(度)
 * @return 是否在限位范围内
 */
bool axis_is_angle_in_limits(axis_handle_t *handle, float angle);

// === 参数配置接口 ===

/**
 * @brief 设置运动参数
 * @param handle 轴控制器句柄
 * @param params 运动参数
 * @return 是否成功
 */
bool axis_set_motion_params(axis_handle_t *handle, const axis_motion_params_t *params);

// === 状态处理接口 ===

/**
 * @brief 轴控制器更新处理（需要在主循环中定期调用）
 * @param handle 轴控制器句柄
 */
void axis_update(axis_handle_t *handle);

/**
 * @brief 重置轴控制器到未初始化状态
 * @param handle 轴控制器句柄
 * @return 是否成功
 */
bool axis_reset(axis_handle_t *handle);

// === 调试和状态字符串接口 ===

/**
 * @brief 获取轴状态字符串
 * @param state 轴状态
 * @return 状态字符串
 */
const char* axis_get_state_string(axis_state_t state);

/**
 * @brief 获取运动结果字符串
 * @param result 运动结果
 * @return 结果字符串
 */
const char* axis_get_result_string(axis_result_t result);

/**
 * @brief 镜像配置说明
 * 
 * mirror_position (位置镜像):
 * - 仅在外部接口的输入输出时进行转换
 * - 内部计算和存储始终使用原始编码器数据
 * - 用于映射码盘定位和物理世界位置的关系
 * - 以限位范围的中心点为镜像轴进行镜像
 * 
 * mirror_motion (运动镜像):
 * - 用于映射电机运动方向和码盘正方向的关系
 * - 运行全程均使用镜像配置进行映射
 * - 影响电机运动方向的计算
 * - 当码盘正方向与期望的电机正方向相反时启用
 */

#ifdef __cplusplus
}
#endif

#endif // __AXIS_CONTROLLER_H__
