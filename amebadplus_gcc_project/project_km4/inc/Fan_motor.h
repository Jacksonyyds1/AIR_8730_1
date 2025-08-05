#ifndef __FAN_MOTOR_H
#define __FAN_MOTOR_H

#include <stdint.h>
#include <stdbool.h>

// 风扇配置参数
#define FAN_FG_PULSE_PER_CYCLE  12    // 每转脉冲数
#define FAN_PWM_FREQ           4000   // PWM频率 4kHz
#define FAN_PWM_MAX_DUTY       4000   // 最大占空比
#define PID_OUTPUT_BIAS        800    // PID输出偏置
#define PID_KP                 0.05f   // PID参数
#define PID_KI                 0.01f
#define PID_KD                 0.08f
#define PID_OUTPUT_MAX         (4000 - PID_OUTPUT_BIAS)
#define PID_OUTPUT_MIN         (0 - PID_OUTPUT_BIAS)

// GPIO和定时器配置
#define FAN_PWM_PIN           _PB_30   // PWM输出引脚
#define FAN_FG_PIN            _PB_31    // FG信号输入引脚


// 函数声明
void fan_speed_controller_init(const int *table, int size);
void fan_speed_set_auto_mode(bool enable);
void fan_speed_set_speed(int speed, bool auto_mode);
void fan_speed_te_set(int rpm);
int fan_speed_get_real_speed(void);
int fan_speed_get_rpm(void);
int fan_speed_get_fg_pps(void);
int fan_speed_get_current_duty(void);

void fan_controller_example(void);
void test_pwm_output(void);
#endif // __FAN_MOTOR_H