#ifndef __ENCODER_MAP_H
#define __ENCODER_MAP_H

#include <stdint.h>
#include <stdbool.h>
#include "absolute_encoder.h"

// 队列大小定义（必须是2的幂，用于位运算优化）
#define ENCODER_QUEUE_SIZE 32
#define ENCODER_QUEUE_MASK (ENCODER_QUEUE_SIZE - 1)

#define ENCODER_UNITS_PER_DEG      1.0f   // 1 encoder unit = 1 step
#define DEGREE_PER_STEP             7.5f   // 7.5 degrees per step
#define STEPPER_MOTOR_GEAR_RATIO    43.2f  // Gear ratio for the stepper motor
#define NECK_MOTOR_GEAR_RATIO       3.5f   // Gear ratio for the neck motor
#define BASE_MOTOR_GEAR_RATIO       5.0f   // Gear ratio for the base motor

#define NECK_MOTOR_STEPS_PER_DEGREE   (STEPPER_MOTOR_GEAR_RATIO * NECK_MOTOR_GEAR_RATIO / DEGREE_PER_STEP)
#define BASE_MOTOR_STEPS_PER_DEGREE   (STEPPER_MOTOR_GEAR_RATIO * BASE_MOTOR_GEAR_RATIO / DEGREE_PER_STEP)

void encoder_init(void);
encoder_handle_t encoder_get_handle(uint8_t motor_index);
int encoder_get_position(uint8_t index);
float encoder_get_estimate_position(uint8_t index);

encoder_state_t encoder_get_state(uint8_t index);
const char *encoder_get_state_string(encoder_state_t state);
encoder_result_t encoder_get_search_result(uint8_t index);
const char *encoder_get_search_result_string(encoder_result_t result);

// 模拟采样接口
void encoder_enable_simulated_signals(bool enable);
uint8_t read_simulated_signal(uint8_t motor_index, int direction);

// 新增的快速采样和处理函数
void encoder_process_samples(void);

// 运行时校准管理函数
void encoder_get_runtime_calibration_status(uint8_t motor_index, runtime_calibration_result_t *result);
void encoder_force_calibration_analysis(uint8_t motor_index);
const char* encoder_get_calibration_state_string(runtime_calibration_state_t state);

#endif // __ENCODER_MAP_H__
