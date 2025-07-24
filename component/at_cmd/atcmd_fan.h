/*
 * 风扇控制AT命令头文件 - 与Fan_motor.c集成版本
 * 文件: atcmd_fan.h
 */

#ifndef __ATCMD_FAN_H__
#define __ATCMD_FAN_H__

#include "ameba_soc.h"
#include "Fan_motor.h"  // 引入现有的风扇控制头文件

// 风扇AT命令配置参数（基于现有Fan_motor.h的定义）
#define FAN_MAX_SPEED_LEVEL    5      // 最大速度档位（0-5，0为停转）
#define FAN_MAX_RPM           2500    // 最大转速限制

// 风扇AT命令操作类型
typedef enum {
    FAN_AT_OP_POWER_OFF = 0,      // 关闭风扇
    FAN_AT_OP_POWER_ON = 1,       // 开启风扇
    FAN_AT_OP_GET_STATUS = 2,     // 获取状态
    FAN_AT_OP_SET_LEVEL = 3,      // 设置速度档位
    FAN_AT_OP_SET_RPM = 4,        // 设置目标转速
    FAN_AT_OP_SET_AUTO_MODE = 5   // 设置自动模式
} fan_at_operation_t;

// 风扇AT命令状态
typedef enum {
    FAN_AT_STATE_OFF = 0,         // 风扇关闭
    FAN_AT_STATE_ON = 1           // 风扇开启
} fan_at_state_t;

// 风扇AT命令控制结构体
typedef struct {
    fan_at_state_t state;         // 当前开关状态
    uint8_t speed_level;          // 当前速度档位 (0-5)
    bool auto_mode;               // 自动模式状态
    bool te_mode;                 // TE模式（直接转速控制）状态
    int target_rpm;               // 目标转速（TE模式）
} fan_at_control_t;

// 函数声明
void at_fan_init(void);
void print_fan_at(void);
void at_fan(void *arg);


#endif /* __ATCMD_FAN_H__ */