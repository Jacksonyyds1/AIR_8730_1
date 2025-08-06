/*
 * SEN68环境传感器AT命令实现 - 头文件
 * 文件: atcmd_sen68.h
 * 功能: 为SEN68传感器提供AT命令接口，支持数据采集和设备控制
 */

#ifndef __ATCMD_SEN68_H__
#define __ATCMD_SEN68_H__

#include "ameba_soc.h"
#include "sen68.h"  // 引入现有的SEN68传感器头文件

// SEN68 AT命令配置参数
#define SEN68_AT_SAMPLE_INTERVAL_MIN    1000    // 最小采样间隔 (ms)
#define SEN68_AT_SAMPLE_INTERVAL_MAX   60000    // 最大采样间隔 (ms)
#define SEN68_AT_SAMPLE_INTERVAL_DEF   10000    // 默认采样间隔 (ms)

// SEN68 AT命令操作类型
typedef enum {
    SEN68_AT_OP_INIT = 0,           // 初始化传感器
    SEN68_AT_OP_START = 1,          // 开始采集数据
    SEN68_AT_OP_STOP = 2,           // 停止采集数据  
    SEN68_AT_OP_STATUS = 3,         // 获取传感器状态
    SEN68_AT_OP_READ_DATA = 4,      // 读取一次数据
    SEN68_AT_OP_READ_CONTINUOUS = 5,// 连续读取数据
    SEN68_AT_OP_SET_INTERVAL = 6,   // 设置采样间隔
    SEN68_AT_OP_RESET = 7,          // 重置传感器
    SEN68_AT_OP_GET_INFO = 8,       // 获取设备信息
    SEN68_AT_OP_SET_OFFSET = 9      // 设置温度补偿偏移
} sen68_at_operation_t;

// SEN68 AT命令状态
typedef enum {
    SEN68_AT_STATE_UNINIT = 0,      // 未初始化
    SEN68_AT_STATE_IDLE = 1,        // 空闲状态
    SEN68_AT_STATE_SAMPLING = 2,    // 正在采样
    SEN68_AT_STATE_CONTINUOUS = 3,  // 连续采样模式
    SEN68_AT_STATE_ERROR = 4        // 错误状态
} sen68_at_state_t;

// SEN68 AT命令控制结构体
typedef struct {
    sen68_at_state_t state;         // 当前状态
    bool auto_mode;                 // 自动采样模式
    uint32_t sample_interval;       // 采样间隔(ms)
    uint32_t last_sample_time;      // 上次采样时间
    uint16_t sample_count;          // 采样计数
    bool data_ready;                // 数据就绪标志
    rtos_task_t continuous_task;    // 连续采样任务句柄
    rtos_timer_t sample_timer;      // 采样定时器
} sen68_at_control_t;

// 数据输出格式类型
typedef enum {
    SEN68_AT_FORMAT_SIMPLE = 0,     // 简单格式（仅数值）
    SEN68_AT_FORMAT_DETAILED = 1,   // 详细格式（包含单位和描述）
    SEN68_AT_FORMAT_JSON = 2        // JSON格式
} sen68_at_format_t;

// 函数声明
void at_sen68_init(void);
void print_sen68_at(void);
void at_sen68(void *arg);

int sen68_at_init_hardware(void);

#endif /* __ATCMD_SEN68_H__ */