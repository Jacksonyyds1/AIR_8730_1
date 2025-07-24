/*
 * 步进电机控制AT命令头文件 - 与step_motor.c集成版本
 * 文件: atcmd_stepper.h
 */

#ifndef __ATCMD_STEPPER_H__
#define __ATCMD_STEPPER_H__

#include "ameba_soc.h"
#include "step_motor.h"  // 引入现有的步进电机控制头文件

// 步进电机AT命令操作类型
typedef enum {
    STEPPER_AT_OP_GET_STATUS = 0,     // 获取状态
    STEPPER_AT_OP_MOVE = 1,           // 移动到位置
    STEPPER_AT_OP_DIRECTION = 2,      // 方向控制
    STEPPER_AT_OP_STOP = 3,           // 停止电机
    STEPPER_AT_OP_RESET_POS = 4,      // 重置位置
    STEPPER_AT_OP_SYNC_MOVE = 5,      // 同步移动
    STEPPER_AT_OP_SET_ACCEL = 6       // 设置加速度
} stepper_at_operation_t;

// 步进电机AT命令状态
typedef enum {
    STEPPER_AT_STATE_STOPPED = 0,     // 停止状态
    STEPPER_AT_STATE_MOVING = 1,      // 移动状态
    STEPPER_AT_STATE_STOPPING = 2     // 停止中状态
} stepper_at_state_t;

// 步进电机AT命令控制结构体
typedef struct {
    stepper_at_state_t motor_state[MOTOR_COUNT];  // 电机状态
    int position[MOTOR_COUNT];                    // 当前位置
    int target_position[MOTOR_COUNT];             // 目标位置
    uint16_t speed[MOTOR_COUNT];                  // 设置速度
    uint8_t accel_rate[MOTOR_COUNT];              // 加速度率
} stepper_at_control_t;

// 速度和位置限制定义
#define STEPPER_MIN_SPEED       10        // 最小速度
#define STEPPER_MAX_SPEED       800      // 最大速度
#define STEPPER_DEFAULT_SPEED   400       // 默认速度
#define STEPPER_MIN_ACCEL       1         // 最小加速度率
#define STEPPER_MAX_ACCEL       255       // 最大加速度率
#define STEPPER_DEFAULT_ACCEL   100       // 默认加速度率

// 位置限制
#define STEPPER_MIN_POSITION    -32768    // 最小位置
#define STEPPER_MAX_POSITION    32767     // 最大位置

// 函数声明
void at_stepper_init(void);
void print_stepper_at(void);
void at_stepper(void *arg);
void at_motor(void *arg);

#endif /* __ATCMD_STEPPER_H__ */