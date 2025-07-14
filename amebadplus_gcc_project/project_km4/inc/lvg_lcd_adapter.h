/*
 * LVGL LCD Adapter Header for RTL8721DCM with ST7789V
 * Optimized for performance and reliability
 */

#ifndef LVGL_LCD_ADAPTER_H
#define LVGL_LCD_ADAPTER_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// LCD 配置常量
#define LCD_W               240
#define LCD_H               320
#define LVGL_TICK_PERIOD    5

// 性能统计结构体
typedef struct {
    uint32_t flush_count;               // 刷新次数
    uint32_t flush_error_count;         // 刷新错误次数  
    uint32_t last_flush_time;           // 上次刷新耗时(ms)
    uint32_t max_flush_time;            // 最大刷新耗时(ms)
    uint32_t total_pixels_transferred;  // 总传输像素数
} lvgl_perf_stats_t;

// 核心初始化和去初始化函数
/**
 * 初始化LVGL和LCD适配器
 * @return 0 成功, -1 失败
 */
int lvgl_init_with_your_lcd(void);

/**
 * 去初始化LVGL
 */
void lvgl_deinit(void);

/**
 * LVGL tick回调函数
 * @return 当前系统时间(ms)
 */
uint32_t lvgl_tick_get_cb(void);

// 演示和测试函数
/**
 * 创建基础UI演示
 */
void create_basic_ui(void);

/**
 * 创建动画演示
 */
void create_animation_demo(void);

/**
 * 测试基础显示功能
 */
void test_basic_display(void);

// 性能监控函数
/**
 * 获取性能统计信息
 * @param stats 输出的统计信息结构体指针
 */
void lvgl_get_performance_stats(lvgl_perf_stats_t *stats);

// 便利宏定义
#define LVGL_COLOR_WHITE    0xFFFF
#define LVGL_COLOR_BLACK    0x0000
#define LVGL_COLOR_RED      0xF800
#define LVGL_COLOR_GREEN    0x07E0
#define LVGL_COLOR_BLUE     0x001F
#define LVGL_COLOR_YELLOW   0xFFE0
#define LVGL_COLOR_CYAN     0x07FF
#define LVGL_COLOR_MAGENTA  0xF81F

// 错误代码定义
#define LVGL_ADAPTER_SUCCESS            0
#define LVGL_ADAPTER_ERROR_INIT         -1
#define LVGL_ADAPTER_ERROR_MEMORY       -2
#define LVGL_ADAPTER_ERROR_HARDWARE     -3
#define LVGL_ADAPTER_ERROR_TASK         -4

#ifdef __cplusplus
}
#endif

#endif // LVGL_LCD_ADAPTER_H