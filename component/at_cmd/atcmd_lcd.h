/*
 * LCD控制AT命令头文件 - 与Lcd.c和lvg_lcd_adapter.c集成版本
 * 文件: atcmd_lcd.h
 */

#ifndef __ATCMD_LCD_H__
#define __ATCMD_LCD_H__

#include "ameba_soc.h"
#include "Lcd.h"  // 引入现有的LCD控制头文件

// LCD AT命令操作类型
typedef enum {
    LCD_AT_OP_GET_STATUS = 0,     // 获取状态
    LCD_AT_OP_BACKLIGHT = 1,      // 背光控制
    LCD_AT_OP_FILL_SCREEN = 2,    // 填充屏幕
    LCD_AT_OP_FILL_RECT = 3,      // 填充矩形
    LCD_AT_OP_FILL_DMA = 4,       // DMA填充
    LCD_AT_OP_LVGL_ENABLE = 5,    // LVGL启用/禁用
    LCD_AT_OP_LVGL_DEMO = 6,      // LVGL演示
    LCD_AT_OP_RESET = 7           // 重置LCD
} lcd_at_operation_t;

// LVGL演示类型
typedef enum {
    LCD_DEMO_LABEL = 0,           // 标签演示
    LCD_DEMO_ANIMATION = 1        // 动画演示
} lcd_demo_type_t;

// LCD AT命令控制结构体
typedef struct {
    bool display_on;              // 显示开关状态
    bool backlight_on;            // 背光开关状态
    uint8_t brightness;           // 亮度百分比 (0-100)
    bool lvgl_enabled;            // LVGL启用状态
    uint16_t last_color;          // 最后使用的颜色
    uint32_t fill_count;          // 填充操作计数
} lcd_at_control_t;

// 常用颜色定义（来自Lcd.h，这里重新列出便于参考）
// RGB565格式颜色值
#define LCD_COLOR_WHITE     0xFFFF    // 白色
#define LCD_COLOR_BLACK     0x0000    // 黑色
#define LCD_COLOR_RED       0xF800    // 红色
#define LCD_COLOR_GREEN     0x0721    // 绿色（修正值）
#define LCD_COLOR_BLUE      0x001F    // 蓝色
#define LCD_COLOR_YELLOW    0xFEE0    // 黄色
#define LCD_COLOR_CYAN      0x7FFF    // 青色
#define LCD_COLOR_MAGENTA   0xF81F    // 洋红色
#define LCD_COLOR_ORANGE    0xFC00    // 橙色
#define LCD_COLOR_PURPLE    0x929F    // 紫色
#define LCD_COLOR_PINK      0xFAB9    // 粉色
#define LCD_COLOR_BROWN     0xBC40    // 棕色
#define LCD_COLOR_GRAY      0x8430    // 灰色

// 屏幕尺寸限制（来自Lcd.h）
#define LCD_AT_MAX_WIDTH    240       // 最大宽度
#define LCD_AT_MAX_HEIGHT   240       // 最大高度

// 亮度控制范围
#define LCD_AT_MIN_BRIGHTNESS   0     // 最小亮度
#define LCD_AT_MAX_BRIGHTNESS   100   // 最大亮度

// 函数声明
void at_lcd_init(void);
void print_lcd_at(void);
void at_lcd(void *arg);
void at_screen(void *arg);

#endif /* __ATCMD_LCD_H__ */