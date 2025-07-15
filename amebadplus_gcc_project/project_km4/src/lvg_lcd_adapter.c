/*
 * LVGL 8.3 Compatible LCD Adapter for Realtek AmebaDPlus
 * Modified to be compatible with LVGL 8.3 API and connect to LCD DMA functions
 */

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "lvgl.h"
#include "os_wrapper.h"
#include "Lcd.h"  // 添加LCD头文件

// LCD 配置参数 - 根据实际LCD修改
#define LCD_W               240
#define LCD_H               240
#define LVGL_TICK_PERIOD    5    // 5ms

// 显示缓冲区 - 根据实际内存情况调整大小
#define BUFFER_LINES 240
static uint8_t g_draw_buf_1[LCD_W * BUFFER_LINES * 2];  // 2 bytes per pixel for RGB565
static uint8_t g_draw_buf_2[LCD_W * BUFFER_LINES * 2];  // 第二个缓冲区，可选

// LVGL 8.3 使用的驱动结构
static lv_disp_drv_t g_disp_drv;       // 显示驱动配置
static lv_disp_draw_buf_t g_disp_buf;  // 绘制缓冲区管理
static lv_disp_t *g_display = NULL;    // 显示设备句柄



// 函数声明
static void lvgl_flush_cb(lv_disp_drv_t *disp_drv, const lv_area_t *area, lv_color_t *color_p);
uint32_t lvgl_tick_get_cb(void);

/**
 * LVGL tick 回调函数
 * 用于获取系统时间，LVGL 8.3 支持自定义tick源
 */
uint32_t lvgl_tick_get_cb(void)
{
    return rtos_time_get_current_system_time_ms();
}

/**
 * 显示刷新回调函数 - LVGL的核心接口
 * 
 * 工作原理：
 * 1. LVGL在内存中绘制UI元素
 * 2. 当需要更新屏幕时，调用这个函数
 * 3. 这个函数负责将内存中的图像数据传输到LCD硬件
 * 参数解释：
 * - disp_drv: 显示驱动对象
 * - area: 需要刷新的屏幕区域（矩形）
 * - color_p: 指向图像数据的指针（RGB565格式）
 */
static void lvgl_flush_cb(lv_disp_drv_t *disp_drv, const lv_area_t *area, lv_color_t *color_p)
{
    // 检查参数有效性
    if (area == NULL || color_p == NULL) {
        lv_disp_flush_ready(disp_drv);
        return;
    }
    
    // 计算显示区域参数
    uint16_t x1 = area->x1;
    uint16_t y1 = area->y1;
    uint16_t x2 = area->x2;
    uint16_t y2 = area->y2;
    uint16_t width = x2 - x1 + 1;
    uint16_t height = y2 - y1 + 1;
    
    // 设置LCD显示窗口
    LCD_Address_Set(x1, y1, x2, y2);
    
    //计算像素数量
    uint32_t pixel_count = width * height;
    
    // 调用LCD的DMA传输函数
    LCD_Write_Buffer_DMA((const uint16_t*)color_p, pixel_count);
  

    // 通知LVGL刷新完成
    lv_disp_flush_ready(disp_drv);
}

/**
 * LVGL任务处理函数
 */
void lvgl_task_handler(void *param)
{
    (void)param;  // 避免未使用参数警告
    
    while (1) {
        // 处理LVGL事件和绘制
        uint32_t time = lv_timer_handler();
        
        // 延时，根据实际需求调整
        rtos_time_delay_ms(time);
    }
}

/**
 * 初始化LVGL和LCD适配器
 * 兼容LVGL 8.3 API，并初始化LCD硬件
 */
int lvgl_init_with_your_lcd(void)
{
    
    // 1. 初始化LVGL
    printf("Initializing LVGL...\n");
    lv_init();
    
    // 2. 初始化绘制缓冲区 (LVGL 8.3 方式)
    lv_disp_draw_buf_init(&g_disp_buf, g_draw_buf_1,g_draw_buf_2 , LCD_W * LCD_H);

    // 3. 初始化显示驱动 (LVGL 8.3 方式)
    lv_disp_drv_init(&g_disp_drv);
    g_disp_drv.hor_res = LCD_W;
    g_disp_drv.ver_res = LCD_H;
    g_disp_drv.flush_cb = lvgl_flush_cb;  // 使用修改后的回调函数
    g_disp_drv.draw_buf = &g_disp_buf;
    
    // 可选：设置全刷新模式
    g_disp_drv.full_refresh = 0;  // 0为部分刷新，1为全刷新
    
    // 4. 注册显示驱动
    g_display = lv_disp_drv_register(&g_disp_drv);
    if (g_display == NULL) {
        printf("Failed to register LVGL display driver\n");
        return -1;
    }
    // 5. 创建LVGL任务
    // 根据实际需求调整堆栈大小和优先级
    if (rtos_task_create(NULL, 
                        "lvgl_task", 
                        lvgl_task_handler, 
                        NULL, 
                        2048, 
                        4) != 0) {  // 使用具体数值而不是tskIDLE_PRIORITY
        printf("Failed to create LVGL task\n");
        return -4;
    }
    
    printf("LVGL 8.3 initialization completed successfully\n");
    return 0;
}

/**
 * 获取显示对象指针
 */
lv_disp_t* lvgl_get_display(void)
{
    return g_display;
}

/**
 * 示例：创建一个简单的标签测试
 */
void lvgl_demo_label(void)
{
    // 1. 设置屏幕背景色（通过LVGL系统）
    lv_obj_t *screen = lv_scr_act();
    lv_obj_set_style_bg_color(screen, lv_color_hex(0x000000), LV_PART_MAIN);  // 黑色背景
    lv_obj_set_style_bg_opa(screen, LV_OPA_COVER, LV_PART_MAIN);  // 完全不透明
    
    // 2. 创建标签
    lv_obj_t *label = lv_label_create(screen);
    lv_label_set_text(label, "Hello LVGL 8.3!\nLCD DMA Connected!");
    lv_obj_align(label, LV_ALIGN_CENTER, 0, 0);
    
    // 3. 设置标签文字颜色为白色（在黑色背景上显示）
    lv_obj_set_style_text_color(label, lv_color_hex(0xFFFFFF), LV_PART_MAIN);
        
    printf("LVGL demo created with black background\n");
}
// 动画回调包装函数
static void anim_x_cb(void *obj, int32_t value)
{
    lv_obj_set_x((lv_obj_t*)obj, (lv_coord_t)value);
}
// =================================================================
// 3. 动画效果
// =================================================================
void create_animation_demo(void)
{
     // 首先设置屏幕背景为黑色
    lv_obj_t *screen = lv_scr_act();
    lv_obj_set_style_bg_color(screen, lv_color_hex(0x000000), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(screen, LV_OPA_COVER, LV_PART_MAIN);
    
    // 创建方块（稍微优化）
    lv_obj_t *box = lv_obj_create(screen);
    lv_obj_set_size(box, 40, 40);  // 稍小一点
    lv_obj_set_style_bg_color(box, lv_color_hex(0xFFFF00), LV_PART_MAIN);
    lv_obj_set_style_border_width(box, 2, LV_PART_MAIN);
    lv_obj_set_style_border_color(box, lv_color_hex(0xFFFFFF), LV_PART_MAIN);
    lv_obj_set_pos(box, 10, 100);

    lv_obj_t *box2 = lv_obj_create(screen);
    lv_obj_set_size(box2, 40, 40);  // 稍小一点
    lv_obj_set_style_bg_color(box2, lv_color_hex(0x0099FF), LV_PART_MAIN);
    lv_obj_set_style_border_width(box2, 2, LV_PART_MAIN);
    lv_obj_set_style_border_color(box2, lv_color_hex(0xFFFFFF), LV_PART_MAIN);
    lv_obj_set_pos(box2, 190, 160);
    
    // 创建动画（更慢更平滑）
    lv_anim_t a;
    lv_anim_init(&a);
    lv_anim_set_var(&a, box);
    lv_anim_set_values(&a, 10, 190);
    lv_anim_set_time(&a, 3000);      // 3秒，更慢
    lv_anim_set_exec_cb(&a, anim_x_cb);
    lv_anim_set_path_cb(&a, lv_anim_path_ease_in_out);
    lv_anim_set_repeat_count(&a, LV_ANIM_REPEAT_INFINITE);
    lv_anim_set_playback_time(&a, 3000);
    lv_anim_start(&a);

    lv_anim_t b;
    lv_anim_init(&b);
    lv_anim_set_var(&b, box2);
    lv_anim_set_values(&b, 190, 10);
    lv_anim_set_time(&b, 3000);      // 3秒，更慢
    lv_anim_set_exec_cb(&b, anim_x_cb);
    lv_anim_set_path_cb(&b, lv_anim_path_ease_in_out);
    lv_anim_set_repeat_count(&b, LV_ANIM_REPEAT_INFINITE);
    lv_anim_set_playback_time(&b, 3000);
    lv_anim_start(&b);
}

/* 如果需要支持触摸屏，可以添加以下代码 */
#ifdef ENABLE_TOUCH_INPUT

// 触摸相关变量
static lv_indev_drv_t g_indev_drv;
static lv_indev_t *g_indev = NULL;

/**
 * 触摸读取回调函数
 */
static void touchpad_read_cb(lv_indev_drv_t *indev_drv, lv_indev_data_t *data)
{
    (void)indev_drv;  // 避免未使用参数警告
    
    // 这里应该实现实际的触摸读取代码
    // 示例框架：
    
    // bool touched = touch_is_pressed();
    // if (touched) {
    //     data->state = LV_INDEV_STATE_PRESSED;
    //     data->point.x = touch_get_x();
    //     data->point.y = touch_get_y();
    // } else {
    //     data->state = LV_INDEV_STATE_RELEASED;
    // }
    
    // 临时代码 - 实际使用时应该替换为真实的触摸读取
    data->state = LV_INDEV_STATE_RELEASED;
    data->point.x = 0;
    data->point.y = 0;
}

/**
 * 初始化触摸输入设备
 */
void lvgl_init_touchpad(void)
{
    // 初始化触摸驱动
    lv_indev_drv_init(&g_indev_drv);
    g_indev_drv.type = LV_INDEV_TYPE_POINTER;
    g_indev_drv.read_cb = touchpad_read_cb;
    
    // 注册触摸设备
    g_indev = lv_indev_drv_register(&g_indev_drv);
}

#endif /* ENABLE_TOUCH_INPUT */