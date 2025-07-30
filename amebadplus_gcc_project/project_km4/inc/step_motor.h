#ifndef __STEPPER_MOTOR_RTL_OPTIMIZED_H__
#define __STEPPER_MOTOR_RTL_OPTIMIZED_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "ameba_soc.h"
#include "gpio_api.h"
#include "os_wrapper.h"
#include <stdbool.h>
#include "platform_autoconf.h" 

// --- 硬件定时器配置 ---
#define MOTOR_TIMER_FREQ        40000000    // RTL8721DCM定时器频率 40MHz
#define MOTOR_MIN_PPS           10          // 最小脉冲频率
#define MOTOR_MAX_PPS           2000        // 最大脉冲频率
#define MOTOR_MIN_PERIOD        20          // 最小定时器周期
#define STEP_TICKS_MAX          1000        // 最大步进计数
#define MOTOR_COUNT             2           // 电机数量

// 加减速配置
#define ACCEL_DECEL_STEPS       100         // 加减速步数
#define DEFAULT_ACCEL_RATE      20          // 默认加速度


#define ENCODER_QUEUE_SIZE_1      20     


// 硬件定时器索引定义
#define MOTOR_TIMER_NECK_IDX    2           // TIM2用于NECK电机
#define MOTOR_TIMER_BASE_IDX    3           // TIM3用于BASE电机

// 电机索引定义
#define MOTOR_NECK              0
#define MOTOR_BASE              1

// GPIO引脚定义 (需根据实际硬件连接修改)
#define MOTOR_NECK_A_PLUS       PA_12
#define MOTOR_NECK_B_PLUS       PA_13
#define MOTOR_NECK_A_MINUS      PA_14
#define MOTOR_NECK_B_MINUS      PA_15

#define MOTOR_BASE_A_PLUS       PB_21
#define MOTOR_BASE_B_PLUS       PB_20
#define MOTOR_BASE_A_MINUS      PB_19
#define MOTOR_BASE_B_MINUS      PB_18

// --- 枚举定义 ---
typedef enum {
    Motor_State_Stop       = 0,
    Motor_State_Starting   = 1,
    Motor_State_Forward    = 2,
    Motor_State_Backward   = 3,
    Motor_State_Stopping   = 4
} Motor_State_t;

typedef enum {
    Motor_Running_Type_Positioning = 0,
    Motor_Running_Type_Direction   = 1
} Motor_Running_Type_t;

typedef enum {
    Motor_Direction_Forward  = 0,
    Motor_Direction_Backward = 1,
    Motor_Direction_Stop     = 2
} Motor_Direction_t;

typedef struct {
    uint8_t motor_index;
    uint8_t sampled_signal;
    Motor_Direction_t direction;
} encoder_sampled_data_t;

// --- 函数声明 ---

// 基础控制函数
Motor_State_t stepper_motor_get_state(uint8_t index);
Motor_Running_Type_t stepper_motor_get_running_type(uint8_t index);
int stepper_motor_get_position(uint8_t index);
void stepper_motor_set_target_position(uint8_t index, int position);
void stepper_motor_set_position(uint8_t index, int position);
void stepper_motor_set_direction(uint8_t index, Motor_Direction_t target_direction, uint16_t target_speed);
void stepper_motor_stop(uint8_t index, bool motor_break, bool emergency);

// 速度和加速度配置函数
void stepper_motor_set_speed(uint8_t index, uint16_t target_speed, bool immediate);
void stepper_motor_set_acceleration_rate(uint8_t index, uint8_t accel_rate);
uint16_t stepper_motor_get_current_speed(uint8_t index);
uint16_t stepper_motor_calc_accel_step(uint8_t index);
void stepper_motor_set_speed_profile(uint8_t index, uint16_t target_speed, uint8_t accel_rate);

// 同步控制函数
void stepper_motor_set_sync_target(int neck_target, int base_target, uint16_t speed);
bool stepper_motor_is_sync_complete(void);
bool stepper_motor_all_stopped(void);

// 编码器数据获取函数
bool stepper_motor_get_encoder_data(encoder_sampled_data_t *data, int timeout);

// 初始化函数
void stepper_motor_init(void);
void stepper_motor_gpio_init(void);
void stepper_motor_timer_init(void);

// 编码器读取函数声明 (需要用户实现)
uint8_t encoder_neck_read(void);
uint8_t encoder_base_read(void);

// --- 硬件定时器API声明 ---
// 注意：以下函数可能需要根据RTL8730 SDK实际API进行调整

/**
 * @brief 改变硬件定时器周期
 * @param TIMx 定时器句柄
 * @param period 新的周期值
 * @note 此函数可能需要根据实际SDK API进行实现
 */
void RTIM_ChangePeriod(RTIM_TypeDef *TIMx, uint32_t period);

// --- 调试和监控函数 ---
#ifdef DEBUG_STEPPER_MOTOR
void stepper_motor_print_status(uint8_t index);
void stepper_motor_print_all_status(void);
#endif

// --- 高级功能函数 (可选实现) ---
typedef struct {
    int start_position;
    int end_position;
    uint16_t max_speed;
    uint16_t accel_rate;
    uint32_t delay_ms;
} stepper_motion_profile_t;

void stepper_motor_execute_motion_profile(uint8_t index, const stepper_motion_profile_t *profile);
bool stepper_motor_is_motion_complete(uint8_t index);

// --- 错误处理 ---
typedef enum {
    STEPPER_ERROR_NONE = 0,
    STEPPER_ERROR_INVALID_INDEX,
    STEPPER_ERROR_TIMER_INIT_FAILED,
    STEPPER_ERROR_GPIO_INIT_FAILED,
    STEPPER_ERROR_QUEUE_CREATE_FAILED,
    STEPPER_ERROR_ENCODER_TIMEOUT
} stepper_error_t;

stepper_error_t stepper_motor_get_last_error(void);
const char* stepper_motor_error_string(stepper_error_t error);

#ifdef __cplusplus
}
#endif

#endif /* __STEPPER_MOTOR_RTL_OPTIMIZED_H__ */