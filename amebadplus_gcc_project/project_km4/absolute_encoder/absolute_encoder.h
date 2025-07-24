#ifndef ABSOLUTE_ENCODER_H
#define ABSOLUTE_ENCODER_H

#include <stdint.h>
#include <stdbool.h>
#include "logger/logger.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 编码器状态枚举
 */
typedef enum
{
    ENCODER_STATE_UNINITIALIZED = 0,   // 未初始化
    ENCODER_STATE_SEARCHING,           // 搜索模式
    ENCODER_STATE_TRACKING,            // 跟踪模式
    ENCODER_STATE_ERROR                // 错误状态
} encoder_state_t;

/**
 * @brief 编码器运行结果
 */
typedef enum
{
    ENCODER_RESULT_OK = 0,             // 正常
    ENCODER_RESULT_SEARCHING,          // 搜索中
    ENCODER_RESULT_POSITION_FOUND,     // 找到位置
    ENCODER_RESULT_TRACKING_UPDATED,   // 跟踪位置更新
    ENCODER_RESULT_ERROR_INVALID_SIGNAL, // 无效信号
    ENCODER_RESULT_ERROR_LOST_TRACKING,  // 跟踪丢失
    ENCODER_RESULT_ERROR_TIMEOUT       // 超时
} encoder_result_t;

/**
 * @brief 编码器配置参数
 */
typedef struct
{
    float motor_steps_per_unit;        // 电机多少步对应编码器前进1个单位
    uint8_t backlash_compensation;     // 回差补偿步数
    uint8_t tracking_lost_threshold;   // 跟踪丢失阈值
} encoder_config_t;

/**
 * @brief 编码器位置信息
 */
typedef struct
{
    uint32_t absolute_position;        // 绝对位置
    int32_t relative_position_change;  // 相对位置变化
    uint8_t confidence_level;          // 置信度等级 (0-100)
    encoder_state_t state;             // 当前状态
} encoder_position_t;

/**
 * @brief 编码器统计信息
 */
typedef struct
{
    uint32_t total_signals_processed;  // 处理的总信号数
    uint32_t search_attempts;          // 搜索尝试次数
    uint32_t tracking_updates;         // 跟踪更新次数
    uint32_t error_count;              // 错误计数
} encoder_stats_t;

/**
 * @brief 运行时校准配置参数
 */
typedef struct
{
    float nominal_steps_per_unit;           // 标称步数每单位
    float tolerance_ratio;                  // 容差比例 (0.0-1.0)
    uint16_t min_samples_for_calibration;   // 校准所需最小样本数
    uint16_t max_samples_for_analysis;      // 分析用最大样本数
} runtime_calibration_config_t;

/**
 * @brief 运行时校准状态
 */
typedef enum
{
    RUNTIME_CALIBRATION_UNCALIBRATED = 0, // 未校准
    RUNTIME_CALIBRATION_COLLECTING,       // 收集数据中
    RUNTIME_CALIBRATION_CALIBRATED,       // 已校准
    RUNTIME_CALIBRATION_FAILED            // 校准失败
} runtime_calibration_state_t;

/**
 * @brief 运行时校准结果信息
 */
typedef struct
{
    runtime_calibration_state_t state;      // 校准状态
    float calibrated_steps_per_unit;        // 校准后的步数每单位
    float confidence_level;                 // 置信度等级 (0.0-1.0)
    uint32_t sample_count;                  // 使用的样本数
    float step_variance;                    // 步数方差
} runtime_calibration_result_t;

/**
 * @brief 采样点配置参数
 */
typedef struct
{
    float sampling_point_ratio;         // 采样点位置比例（0.0-1.0）
    uint32_t max_steps_for_edge;        // 等待边沿的最大步数
} encoder_sampling_config_t;

// 不透明句柄类型
typedef struct encoder_handle_s *encoder_handle_t;
typedef struct encoder_map_handle_s *encoder_map_handle_t;

// 日志级别
extern int absolute_encoder_log_level;

/**
 * @brief 创建编码器图案对象
 * @param pattern_data 编码器图案数据
 * @param pattern_length 图案长度
 * @param max_search_depth 最大搜索深度
 * @return 编码器图案句柄，失败返回NULL
 */
encoder_map_handle_t abs_encoder_create_map(const uint8_t *pattern_data, uint32_t pattern_length, uint8_t max_search_depth);

/**
 * @brief 创建编码器图案对象（使用内存池）
 * @param pattern_data 编码器图案数据
 * @param pattern_length 图案长度
 * @param max_search_depth 最大搜索深度
 * @param node_pool 内存池指针
 * @param pool_size 内存池大小
 * @return 编码器图案句柄，失败返回NULL
 */
encoder_map_handle_t abs_encoder_create_map_with_pool(const uint8_t *pattern_data, uint32_t pattern_length, uint8_t max_search_depth, void *node_pool, uint16_t pool_size);

/**
 * @brief 销毁编码器图案对象
 * @param map_handle 编码器图案句柄
 */
void abs_encoder_destroy_map(encoder_map_handle_t map_handle);

/**
 * @brief 创建编码器对象
 * @param map_handle 编码器图案句柄
 * @param config 编码器配置
 * @return 编码器句柄，失败返回NULL
 */
encoder_handle_t abs_encoder_create(encoder_map_handle_t map_handle, const encoder_config_t *config);

/**
 * @brief 销毁编码器对象
 * @param handle 编码器句柄
 */
void abs_encoder_destroy(encoder_handle_t handle);

/**
 * @brief 处理电机步进和传感器信号（推荐接口）
 * @param handle 编码器句柄
 * @param direction 方向 (1=前进, -1=后退)
 * @param sensor_signal 传感器信号 (0或1)
 * @param position_info 输出位置信息
 * @return 处理结果
 */
encoder_result_t abs_encoder_process_step_with_signal(encoder_handle_t handle, int8_t direction,
        uint8_t sensor_signal, encoder_position_t *position_info);

/**
 * @brief 获取当前位置信息
 * @param handle 编码器句柄
 * @param position_info 输出位置信息
 * @return 是否成功
 */
bool abs_encoder_get_current_position(encoder_handle_t handle, encoder_position_t *position_info);

/**
 * @brief 重置编码器状态
 * @param handle 编码器句柄
 * @return 是否成功
 */
bool abs_encoder_reset(encoder_handle_t handle);

/**
 * @brief 设置日志级别
 * @param level 日志级别
 * @return 无
 */
void abs_encoder_set_log_level(int level);

/**
 * @brief 配置运行时校准参数
 * @param handle 编码器句柄
 * @param config 运行时校准配置
 * @return 是否成功
 */
bool abs_encoder_config_runtime_calibration(encoder_handle_t handle, const runtime_calibration_config_t *config);

/**
 * @brief 向校准器提供原始步进数据（推荐接口）
 * @param handle 编码器句柄
 * @param sensor_signal 传感器信号 (0或1)
 * @param is_forward 是否前进方向
 * @return 是否成功
 */
bool abs_encoder_add_calibration_step_data(encoder_handle_t handle, uint8_t sensor_signal);

/**
 * @brief 获取运行时校准结果
 * @param handle 编码器句柄
 * @param result 输出校准结果信息
 * @return 是否成功
 */
bool abs_encoder_get_runtime_calibration_result(encoder_handle_t handle, runtime_calibration_result_t *result);

/**
 * @brief 配置采样点参数
 * @param handle 编码器句柄
 * @param sampling_ratio 采样点位置比例（0.0-1.0，默认0.5表示在编码器单位的中点采样）
 * @param max_steps 等待编码器单位完成的最大步数
 * @return 是否成功
 */
bool abs_encoder_config_sampling_point(encoder_handle_t handle, float sampling_ratio, uint16_t symertric_compensation, uint32_t max_steps);

/**
 * @brief 重置运行时校准
 * @param handle 编码器句柄
 * @return 是否成功
 */
bool abs_encoder_reset_runtime_calibration(encoder_handle_t handle);

/**
 * @brief 强制执行运行时校准分析
 * @param handle 编码器句柄
 * @return 是否成功
 */
bool abs_encoder_force_runtime_calibration_analysis(encoder_handle_t handle);

/**
 * @brief 检查运行时校准状态
 * @param handle 编码器句柄
 * @return 校准状态
 */
runtime_calibration_state_t abs_encoder_get_runtime_calibration_state(encoder_handle_t handle);

/**
 * @brief 获取当前采样点配置
 * @param handle 编码器句柄
 * @param config 采样点配置信息
 * @return 是否成功
 */
bool abs_encoder_get_sampling_config(encoder_handle_t handle, encoder_sampling_config_t *config);

/**
 * @brief 检查是否需要重新校准
 * @param handle 编码器句柄
 * @return 是否需要重新校准
 */
bool abs_encoder_needs_recalibration(encoder_handle_t handle);

#ifdef __cplusplus
}
#endif

#endif // ABSOLUTE_ENCODER_H
