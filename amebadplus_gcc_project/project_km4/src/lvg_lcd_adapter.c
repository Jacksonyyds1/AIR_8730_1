/*
 * LVGL 8.3 Compatible LCD Adapter for Realtek AmebaDPlus
 * Modified to be compatible with LVGL 8.3 API
 */

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "lvgl.h"
#include "os_wrapper.h"

// LCD 配置参数 - 根据实际LCD修改
#define LCD_W               240
#define LCD_H               240
#define LVGL_TICK_PERIOD    5    // 5ms

// 显示缓冲区 - 根据实际内存情况调整大小
static uint8_t g_draw_buf_1[LCD_W * LCD_H * 2];  // 2 bytes per pixel for RGB565
static uint8_t g_draw_buf_2[LCD_W * LCD_H * 2];  // 第二个缓冲区，可选

// LVGL 8.3 使用的驱动结构
static lv_disp_drv_t g_disp_drv;
static lv_disp_draw_buf_t g_disp_buf;
static lv_disp_t *g_display = NULL;

// 定时器相关
static rtos_timer_t g_lvgl_tick_timer = NULL;

// 函数声明
static void lvgl_flush_cb(lv_disp_drv_t *disp_drv, const lv_area_t *area, lv_color_t *color_p);
static void lvgl_tick_timer_callback(void *arg);
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
 * 定时器回调函数
 * 如果不使用 LV_TICK_CUSTOM，需要定期调用此函数
 */
static void lvgl_tick_timer_callback(void *arg)
{
    (void)arg;  // 避免未使用参数警告
    // 在LVGL 8.3中，如果配置了LV_TICK_CUSTOM，则不需要手动调用lv_tick_inc
    // 如果没有配置LV_TICK_CUSTOM，则需要取消注释下面的行
    // lv_tick_inc(LVGL_TICK_PERIOD);
}

/**
 * 显示刷新回调函数
 * 在LVGL 8.3中，参数是 lv_disp_drv_t 而不是 lv_display_t
 */
static void lvgl_flush_cb(lv_disp_drv_t *disp_drv, const lv_area_t *area, lv_color_t *color_p)
{
    (void)area;    // 避免未使用参数警告
    (void)color_p; // 避免未使用参数警告
    
    /* 实际的LCD驱动代码应该放在这里 */
    /* 例如：通过SPI或并行接口发送数据到LCD */
    
    // 示例代码框架：
    // 1. 设置LCD的显示窗口
    // lcd_set_window(area->x1, area->y1, area->x2, area->y2);
    
    // 2. 发送像素数据到LCD
    // int32_t pixel_count = (area->x2 - area->x1 + 1) * (area->y2 - area->y1 + 1);
    // lcd_send_data((uint8_t*)color_p, pixel_count * 2);  // RGB565 = 2 bytes per pixel
    
    // 3. 通知LVGL刷新完成
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
        lv_timer_handler();
        
        // 延时，根据实际需求调整
        rtos_time_delay_ms(5);
    }
}

/**
 * 初始化LVGL和LCD适配器
 * 兼容LVGL 8.3 API
 */
void lvgl_init_with_your_lcd(void)
{
    // 1. 初始化LVGL
    lv_init();
    
    // 2. 初始化绘制缓冲区 (LVGL 8.3 方式)
    lv_disp_draw_buf_init(&g_disp_buf, g_draw_buf_1, g_draw_buf_2, LCD_W * LCD_H);
    
    // 3. 初始化显示驱动 (LVGL 8.3 方式)
    lv_disp_drv_init(&g_disp_drv);
    g_disp_drv.hor_res = LCD_W;
    g_disp_drv.ver_res = LCD_H;
    g_disp_drv.flush_cb = lvgl_flush_cb;
    g_disp_drv.draw_buf = &g_disp_buf;
    
    // 可选：设置全刷新模式
    g_disp_drv.full_refresh = 0;  // 0为部分刷新，1为全刷新
    
    // 4. 注册显示驱动
    g_display = lv_disp_drv_register(&g_disp_drv);
    
    // 5. 设置tick源
    // 方式1：使用自定义tick源 (推荐，需要在lv_conf.h中配置LV_TICK_CUSTOM)
    // 如果lv_conf.h中已配置LV_TICK_CUSTOM，则不需要额外设置
    
    // 方式2：使用定时器定期调用 lv_tick_inc (如果没有配置LV_TICK_CUSTOM)
    if (rtos_timer_create(&g_lvgl_tick_timer, 
                         "lvgl_tick", 
                         0,  // timer_id
                         LVGL_TICK_PERIOD, 
                         1,  // 1 = 自动重载
                         lvgl_tick_timer_callback) != 0) {
        printf("Failed to create LVGL tick timer\n");
        return;
    }
    
    // 6. 启动定时器
    if (rtos_timer_start(g_lvgl_tick_timer, 0) != 0) {
        printf("Failed to start LVGL tick timer\n");
        return;
    }
    
    // 7. 创建LVGL任务
    // 根据实际需求调整堆栈大小和优先级
    if (rtos_task_create(NULL, 
                        "lvgl_task", 
                        lvgl_task_handler, 
                        NULL, 
                        2048, 
                        1 + 2) != 0) {  // 使用具体数值而不是tskIDLE_PRIORITY
        printf("Failed to create LVGL task\n");
        return;
    }
    
    printf("LVGL 8.3 initialization completed successfully\n");
}



/**
 * 获取显示对象指针
 */
lv_disp_t* lvgl_get_display(void)
{
    return g_display;
}

/**
 * 清理资源
 */
void lvgl_deinit(void)
{
    if (g_lvgl_tick_timer != NULL) {
        rtos_timer_delete(g_lvgl_tick_timer, 0);
        g_lvgl_tick_timer = NULL;
    }
    
    // 这里可以添加更多清理代码
    // 例如删除任务、释放内存等
}

/**
 * 示例：创建一个简单的标签测试
 */
void lvgl_demo_label(void)
{
    lv_obj_t *label = lv_label_create(lv_scr_act());
    lv_label_set_text(label, "Hello LVGL 8.3!");
    lv_obj_align(label, LV_ALIGN_CENTER, 0, 0);
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