//=================================================================
// microstep_controller.h - 微步控制头文件
//=================================================================

#ifndef __MICROSTEP_CONTROLLER_H__
#define __MICROSTEP_CONTROLLER_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

//=================================================================
// 配置参数
//=================================================================

// 硬件配置选择
#define USE_PWM_CONTROL     1    // 0=GPIO控制(兼容现有), 1=PWM控制(需要硬件升级)

// 微步分辨率配置
#define MICROSTEP_RESOLUTION 64  // 每个全步的微步数 (推荐: 16, 32, 64)

// 性能参数
#define MAX_SPEED_PPS       10000  // 最大速度 (脉冲每秒)
#define MIN_STEP_DELAY_US   100    // 最小步进延迟 (微秒)
#define PWM_FREQUENCY_HZ    20000  // PWM频率 (Hz)

//=================================================================
// 数据结构定义
//=================================================================

/**
 * @brief 电流等级结构 (用于PWM控制)
 */
typedef struct {
    uint8_t a_plus;     // A+相电流等级 (0-255)
    uint8_t b_plus;     // B+相电流等级 (0-255)
    uint8_t a_minus;    // A-相电流等级 (0-255)
    uint8_t b_minus;    // B-相电流等级 (0-255)
} current_levels_t;

/**
 * @brief 微步控制器状态信息
 */
typedef struct {
    int32_t current_position;    // 当前微步位置
    int32_t target_position;     // 目标微步位置
    uint16_t current_speed_pps;  // 当前速度 (脉冲每秒)
    bool is_running;             // 运行状态
    bool is_initialized;         // 初始化状态
    float current_angle_deg;     // 当前角度 (度)
} microstep_status_t;

//=================================================================
// 核心功能接口
//=================================================================

/**
 * @brief 初始化真正的微步控制器
 * @note 自动检测配置，初始化PWM或驱动IC控制
 */
void true_microstep_init(void);

/**
 * @brief 移动到指定微步位置 (绝对定位)
 * @param target_position 目标微步位置
 * @param speed_pps 运动速度 (脉冲每秒, 1-10000)
 */
void true_microstep_move_to_position(int32_t target_position, uint16_t speed_pps);

/**
 * @brief 相对移动指定微步数
 * @param steps 步数 (正数=正向, 负数=反向)
 * @param speed_pps 运动速度 (脉冲每秒)
 */
void true_microstep_move_steps(int32_t steps, uint16_t speed_pps);

/**
 * @brief 停止微步运动
 * @param hold_position true=保持位置(有电流), false=断电释放
 */
void true_microstep_stop(bool hold_position);

//=================================================================
// 状态查询接口
//=================================================================

/**
 * @brief 检查是否正在运行
 * @return true=运行中, false=停止
 */
bool true_microstep_is_running(void);

/**
 * @brief 获取当前微步位置
 * @return 当前微步位置
 */
int32_t true_microstep_get_position(void);

/**
 * @brief 获取当前角度 (度)
 * @return 当前角度值
 */
float true_microstep_get_angle_degrees(void);

/**
 * @brief 获取完整状态信息
 * @param status 状态结构指针
 */
void true_microstep_get_status(microstep_status_t *status);

//=================================================================
// 配置和校准接口
//=================================================================

/**
 * @brief 设置当前位置 (用于零点校准)
 * @param position 设定的位置值
 */
void true_microstep_set_position(int32_t position);

/**
 * @brief 零点校准
 * @note 将当前位置设为0点
 */
static inline void true_microstep_set_zero(void) {
    true_microstep_set_position(0);
}

/**
 * @brief 设置最大速度限制
 * @param max_speed_pps 最大速度 (脉冲每秒)
 */
void true_microstep_set_max_speed(uint16_t max_speed_pps);

//=================================================================
// 高级功能接口
//=================================================================

/**
 * @brief 角度控制 (度为单位)
 * @param target_angle_deg 目标角度 (度)
 * @param speed_dps 角速度 (度每秒)
 */
void true_microstep_move_to_angle(float target_angle_deg, float speed_dps);

/**
 * @brief 相对角度移动
 * @param angle_deg 相对角度 (度)
 * @param speed_dps 角速度 (度每秒)
 */
void true_microstep_rotate_angle(float angle_deg, float speed_dps);

/**
 * @brief 连续旋转控制
 * @param speed_rpm 转速 (转每分钟, 正数=正向, 负数=反向, 0=停止)
 */
void true_microstep_continuous_rotation(float speed_rpm);

//=================================================================
// 测试和演示接口
//=================================================================

/**
 * @brief 微步精度测试
 * @note 慢速转动一个完整步进，验证微步精度
 */
void true_microstep_precision_test(void);

/**
 * @brief 速度测试
 * @note 测试不同速度下的性能表现
 */
void true_microstep_speed_test(void);

/**
 * @brief 平滑度演示
 * @note 演示微步控制的平滑旋转效果
 */
void true_microstep_smoothness_demo(void);

/**
 * @brief 显示电流波形 (调试用)
 * @note 打印微步表的电流分配情况
 */
void true_microstep_show_current_waveform(void);

/**
 * @brief 完整演示程序
 * @note 运行所有测试和演示
 */
void true_microstep_complete_demo(void);

//=================================================================
// 兼容性接口 (保持与现有代码兼容)
//=================================================================

/**
 * @brief 兼容接口：初始化
 */
void enhanced_microstep_init(void);

/**
 * @brief 兼容接口：移动步数 (兼容原有接口)
 * @param steps 步数 (基于原有步进单位)
 * @param speed_ms 每步延迟毫秒数
 */
void enhanced_microstep_move_steps(int32_t steps, uint16_t speed_ms);

/**
 * @brief 兼容接口：停止
 */
static inline void enhanced_microstep_stop(void) {
    true_microstep_stop(false);
}

/**
 * @brief 兼容接口：检查运行状态
 */
static inline bool enhanced_microstep_is_running(void) {
    return true_microstep_is_running();
}

/**
 * @brief 兼容接口：获取位置 (转换为原有单位)
 */
static inline int32_t enhanced_microstep_get_position(void) {
    return true_microstep_get_position() * 4 / MICROSTEP_RESOLUTION;
}

//=================================================================
// 实用宏定义
//=================================================================

// 角度转换
#define DEGREES_TO_MICROSTEPS(deg)  ((int32_t)((deg) * MICROSTEP_RESOLUTION / 1.8f))
#define MICROSTEPS_TO_DEGREES(steps) ((float)(steps) * 1.8f / MICROSTEP_RESOLUTION)

// 转速转换 (RPM → PPS)
#define RPM_TO_PPS(rpm)             ((uint16_t)((rpm) * 200 * MICROSTEP_RESOLUTION / 60))
#define PPS_TO_RPM(pps)             ((float)(pps) * 60 / (200 * MICROSTEP_RESOLUTION))

// 常用位置定义
#define FULL_STEP_MICROSTEPS        MICROSTEP_RESOLUTION
#define HALF_STEP_MICROSTEPS        (MICROSTEP_RESOLUTION / 2)
#define QUARTER_STEP_MICROSTEPS     (MICROSTEP_RESOLUTION / 4)
#define ONE_REVOLUTION_MICROSTEPS   (200 * MICROSTEP_RESOLUTION)

//=================================================================
// 硬件相关配置 (根据实际连接修改)
//=================================================================

// PWM模式引脚定义
#if USE_PWM_CONTROL
    #define MOTOR_A_PLUS_PIN    PB_21  // A+相PWM输出
    #define MOTOR_B_PLUS_PIN    PB_20  // B+相PWM输出
    #define MOTOR_A_MINUS_PIN   PB_30  // A-相PWM输出
    #define MOTOR_B_MINUS_PIN   PB_18  // B-相PWM输出
#else
    // 专用驱动IC模式引脚定义
    #define STEPPER_STEP_PIN    PB_21  // 步进脉冲
    #define STEPPER_DIR_PIN     PB_20  // 方向控制
    #define STEPPER_ENABLE_PIN  PB_19  // 使能控制
    #define STEPPER_MS1_PIN     PB_18  // 微步选择1 (可选)
#endif

//=================================================================
// 错误代码定义
//=================================================================

typedef enum {
    MICROSTEP_OK = 0,
    MICROSTEP_ERROR_NOT_INITIALIZED,
    MICROSTEP_ERROR_INVALID_PARAMETER,
    MICROSTEP_ERROR_HARDWARE_FAULT,
    MICROSTEP_ERROR_SPEED_TOO_HIGH,
    MICROSTEP_ERROR_POSITION_OUT_OF_RANGE
} microstep_error_t;

//=================================================================
// 调试和诊断接口
//=================================================================

/**
 * @brief 获取错误状态
 * @return 错误代码
 */
microstep_error_t true_microstep_get_error(void);

/**
 * @brief 清除错误状态
 */
void true_microstep_clear_error(void);

/**
 * @brief 获取版本信息
 * @return 版本字符串
 */
const char* true_microstep_get_version(void);

/**
 * @brief 获取配置信息
 * @param info_buffer 信息缓冲区
 * @param buffer_size 缓冲区大小
 */
void true_microstep_get_config_info(char *info_buffer, size_t buffer_size);

#ifdef __cplusplus
}
#endif

#endif /* __MICROSTEP_CONTROLLER_H__ */