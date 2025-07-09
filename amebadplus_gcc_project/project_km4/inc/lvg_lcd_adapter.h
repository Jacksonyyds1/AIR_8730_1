#ifndef LVG_LCD_ADAPTER_H
#define LVG_LCD_ADAPTER_H


// LVGL初始化
int lvgl_init_with_your_lcd(void);

// LVGL去初始化
void lvgl_deinit(void);

// 创建演示UI
void create_demo_ui(void);

#endif // LVGL_LCD_ADAPTER_H