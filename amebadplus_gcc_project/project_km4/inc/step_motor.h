#ifndef __STEPPER_MOTOR_RTL_H__
#define __STEPPER_MOTOR_RTL_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "ameba_soc.h"
#include "gpio_api.h"
#include "os_wrapper.h"
#include <stdbool.h>
#include "platform_autoconf.h" 

// --- 步进电机定义 ---
#define MOTOR_TIMER_FREQ    32768       // RTL8721DCM RTIM频率 32.768kHz (低功耗定时器)
#define MOTOR_MIN_PPS       10          // 最小脉冲频率
#define STEP_TICKS_MAX      1000        // 最大步进计数
#define MOTOR_COUNT         2           // 电机数量

// 电机索引定义
#define MOTOR_NECK          0
#define MOTOR_BASE          1

// GPIO引脚定义 (需根据实际硬件连接修改)
#define MOTOR_NECK_A_PLUS     PA_12
#define MOTOR_NECK_B_PLUS     PA_13
#define MOTOR_NECK_A_MINUS    PA_14
#define MOTOR_NECK_B_MINUS    PA_15

#define MOTOR_BASE_A_PLUS       PB_21
#define MOTOR_BASE_B_PLUS       PB_20
#define MOTOR_BASE_A_MINUS      PB_19
#define MOTOR_BASE_B_MINUS      PB_18

// 定时器定义 - 使用RTIM (Real Time Timer)
#define MOTOR_TIMER_NECK        2
#define MOTOR_TIMER_BASE        3

// RTIM相关常量定义
#ifndef GTIMER_ONESHOT_US
#define GTIMER_ONESHOT_US       500000  // 500ms默认周期，用于兼容性
#endif

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

// 配置函数
void stepper_motor_set_speed_profile(uint8_t index, uint16_t target_speed, uint8_t accel_rate);
void stepper_motor_set_speed(uint8_t index, uint16_t target_speed, bool immediate);
void stepper_motor_set_acceleration_rate(uint8_t index, uint8_t accel_rate);

// 状态查询函数
uint16_t stepper_motor_get_current_speed(uint8_t index);
uint16_t stepper_motor_calc_accel_step(uint8_t index);

// 同步控制函数
void stepper_motor_set_sync_target(int nozzle_target, int base_target, uint16_t speed);
bool stepper_motor_is_sync_complete(void);
bool stepper_motor_all_stopped(void);

// 初始化函数
void stepper_motor_init(void);
void stepper_motor_gpio_init(void);
void stepper_motor_timer_init(void);

// 编码器数据获取函数
bool stepper_motor_get_encoder_data(encoder_sampled_data_t *data, int timeout);

void stepper_motor_move(uint8_t index, Motor_Direction_t target_direction, uint16_t target_pps, bool immediate);

#ifdef __cplusplus
}
#endif

#endif /* __STEPPER_MOTOR_RTL_H__ */