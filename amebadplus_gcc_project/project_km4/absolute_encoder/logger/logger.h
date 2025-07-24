#ifndef LOGGER_H
#define LOGGER_H

#include "ameba_soc.h"
#include <stdio.h>
#include <inttypes.h>  // 添加这个头文件以支持PRId32等宏

#ifdef __cplusplus
extern "C" {
#endif

// 使用RTL8721CDM原生的日志函数
// DiagPrintf是RTL8721CDM SDK提供的调试打印函数
// printf也可以正常使用

// 定义日志级别
extern int absolute_encoder_log_level;

// 日志级别定义
#define LOG_LEVEL_NONE    0
#define LOG_LEVEL_ERROR   1
#define LOG_LEVEL_WARN    2
#define LOG_LEVEL_INFO    3
#define LOG_LEVEL_DEBUG   4
#define LOG_LEVEL_VERBOSE 5

// 简单的日志宏，直接使用printf或DiagPrintf
// 使用强制类型转换来避免格式化警告
#define LOGE(fmt, ...) \
    do { \
        if (absolute_encoder_log_level >= LOG_LEVEL_ERROR) { \
            printf("[ABS_ENC_ERR] " fmt "\n", ##__VA_ARGS__); \
        } \
    } while(0)

#define LOGW(fmt, ...) \
    do { \
        if (absolute_encoder_log_level >= LOG_LEVEL_WARN) { \
            printf("[ABS_ENC_WARN] " fmt "\n", ##__VA_ARGS__); \
        } \
    } while(0)

#define LOGI(fmt, ...) \
    do { \
        if (absolute_encoder_log_level >= LOG_LEVEL_INFO) { \
            printf("[ABS_ENC_INFO] " fmt "\n", ##__VA_ARGS__); \
        } \
    } while(0)

#define LOGD(fmt, ...) \
    do { \
        if (absolute_encoder_log_level >= LOG_LEVEL_DEBUG) { \
            printf("[ABS_ENC_DEBUG] " fmt "\n", ##__VA_ARGS__); \
        } \
    } while(0)

#define LOGV(fmt, ...) \
    do { \
        if (absolute_encoder_log_level >= LOG_LEVEL_VERBOSE) { \
            printf("[ABS_ENC_VERBOSE] " fmt "\n", ##__VA_ARGS__); \
        } \
    } while(0)

// 如果你需要处理特定的int32_t格式化，可以使用这些辅助宏
#define LOG_INT32(val) ((int)(val))  // 强制转换为int避免格式化警告

// 如果你更喜欢使用DiagPrintf，可以替换上面的printf为DiagPrintf
// 例如：DiagPrintf("[ABS_ENC_ERR] " fmt "\n", ##__VA_ARGS__)

#ifdef __cplusplus
}
#endif

#endif // LOGGER_H