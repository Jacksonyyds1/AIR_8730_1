#ifndef LVG_LCD_ADAPTER_H
#define LVG_LCD_ADAPTER_H


// LVGL初始化
int lvgl_init_with_your_lcd(void);

// LVGL去初始化
void lvgl_deinit(void);

// 创建演示标签
void lvgl_demo_label(void);

void create_basic_ui(void);

void create_animation_demo(void);

void test_basic_display(void);

#endif // LVGL_LCD_ADAPTER_H