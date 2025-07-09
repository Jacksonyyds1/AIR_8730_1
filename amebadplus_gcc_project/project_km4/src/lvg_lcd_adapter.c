/* 
 * LVGL适配器 - 使用你现有的Lcd.c驱动
 * 文件名: lvgl_lcd_adapter.c
 */

#include "ameba_soc.h"
#include "os_wrapper.h"
#include "lvgl.h"
#include "Lcd.h"  
#include "lvg_lcd_adapter.h"

/*====================
   配置定义
 *====================*/
#define LVGL_TICK_PERIOD    1    // LVGL时钟周期(ms)
#define LVGL_REFRESH_PERIOD 33   // 刷新周期(ms) - 约30FPS

/*====================
   全局变量
 *====================*/
static lv_display_t *g_display = NULL;
static lv_color_t *g_draw_buf_1 = NULL;
static lv_color_t *g_draw_buf_2 = NULL;
static rtos_timer_t lvgl_tick_timer;
static rtos_task_t lvgl_task_handle;

/*====================
   LVGL时钟函数
 *====================*/
static uint32_t lvgl_tick_get_cb(void)
{
    return rtos_time_get_current_system_time_ms();
}

static void lvgl_tick_timer_callback(void *arg)
{
    UNUSED(arg);
    lv_tick_inc(LVGL_TICK_PERIOD);
}

/*====================
   LVGL显示刷新函数 - 适配你的LCD驱动
 *====================*/
static void lvgl_flush_cb(lv_display_t *disp, const lv_area_t *area, uint8_t *px_map)
{
    int32_t x1 = area->x1;
    int32_t y1 = area->y1;
    int32_t x2 = area->x2;
    int32_t y2 = area->y2;
    
    // 计算区域大小
    uint32_t width = x2 - x1 + 1;
    uint32_t height = y2 - y1 + 1;
    uint32_t pixel_count = width * height;
    
    // 使用你现有的DMA函数显示图像
    // px_map已经是RGB565格式的数据，直接使用
    LCD_Display_Image_DMA(x1, y1, width, height, (uint16_t*)px_map);
    
    // 通知LVGL刷新完成
    lv_display_flush_ready(disp);
}

/*====================
   LVGL任务
 *====================*/
static void lvgl_task(void *param)
{
    UNUSED(param);
    
    printf("LVGL task started\n");
    
    while (1) {
        // 处理LVGL任务
        uint32_t time_till_next = lv_timer_handler();
        
        // 如果没有待处理任务，延时到下次处理
        if (time_till_next == LV_NO_TIMER_READY) {
            rtos_time_delay_ms(LVGL_REFRESH_PERIOD);
        } else {
            // 有任务需要处理，但最少延时1ms
            rtos_time_delay_ms(LV_MIN(time_till_next, LVGL_REFRESH_PERIOD));
        }
    }
}

/*====================
   LVGL初始化函数
 *====================*/
int lvgl_init_with_your_lcd(void)
{
    printf("Initializing LVGL with existing LCD driver...\n");
    
    // 1. 初始化LVGL
    lv_init();
    
    // 2. 设置时钟回调
    lv_tick_set_cb(lvgl_tick_get_cb);
    
    // 3. 分配显示缓冲区
    // 注意：这里使用你定义的LCD尺寸
    uint32_t buf_size = LCD_W * LCD_H;
    
    g_draw_buf_1 = (lv_color_t*)rtos_mem_malloc(buf_size * sizeof(lv_color_t));
    g_draw_buf_2 = (lv_color_t*)rtos_mem_malloc(buf_size * sizeof(lv_color_t));
    
    if (g_draw_buf_1 == NULL || g_draw_buf_2 == NULL) {
        printf("Failed to allocate LVGL display buffers\n");
        return -1;
    }
    
    printf("LVGL buffers allocated: %d x %d pixels, %lu bytes each\n", 
           LCD_W, LCD_H, buf_size * sizeof(lv_color_t));
    
    // 4. 创建显示驱动
    g_display = lv_display_create(LCD_W, LCD_H);
    if (g_display == NULL) {
        printf("Failed to create LVGL display\n");
        return -1;
    }
    
    // 5. 设置显示回调和缓冲区
    lv_display_set_flush_cb(g_display, lvgl_flush_cb);
    lv_display_set_buffers(g_display, g_draw_buf_1, g_draw_buf_2, 
                          buf_size * sizeof(lv_color_t), 
                          LV_DISPLAY_RENDER_MODE_PARTIAL);
    
    // 6. 创建定时器用于LVGL时钟
    if (rtos_timer_create(&lvgl_tick_timer, 
                         "LVGL_Tick", 
                         LVGL_TICK_PERIOD,
                         1,  // 周期性定时器
                         lvgl_tick_timer_callback,
                         NULL) != RTK_SUCCESS) {
        printf("Failed to create LVGL tick timer\n");
        return -1;
    }
    
    // 7. 启动定时器
    if (rtos_timer_start(lvgl_tick_timer, 0) != RTK_SUCCESS) {
        printf("Failed to start LVGL tick timer\n");
        return -1;
    }
    
    // 8. 创建LVGL任务
    if (rtos_task_create(&lvgl_task_handle,
                        "LVGL_Task",
                        lvgl_task,
                        NULL,
                        8192,  // 8KB栈空间
                        tskIDLE_PRIORITY + 2) != RTK_SUCCESS) {
        printf("Failed to create LVGL task\n");
        return -1;
    }
    
    printf("LVGL initialized successfully with your LCD driver\n");
    return 0;
}

/*====================
   清理函数
 *====================*/
void lvgl_deinit(void)
{
    // 停止定时器
    if (lvgl_tick_timer) {
        rtos_timer_stop(lvgl_tick_timer, 0);
        rtos_timer_delete(lvgl_tick_timer, 0);
        lvgl_tick_timer = NULL;
    }
    
    // 删除任务
    if (lvgl_task_handle) {
        rtos_task_delete(lvgl_task_handle);
        lvgl_task_handle = NULL;
    }
    
    // 释放缓冲区
    if (g_draw_buf_1) {
        rtos_mem_free(g_draw_buf_1);
        g_draw_buf_1 = NULL;
    }
    
    if (g_draw_buf_2) {
        rtos_mem_free(g_draw_buf_2);
        g_draw_buf_2 = NULL;
    }
    
    printf("LVGL deinitialized\n");
}

/*====================
   示例UI创建函数
 *====================*/
void create_demo_ui(void)
{
    // 创建一个标签
    lv_obj_t *label = lv_label_create(lv_scr_act());
    lv_label_set_text(label, "Hello LVGL!");
    lv_obj_set_style_text_color(label, lv_color_white(), LV_PART_MAIN);
    lv_obj_align(label, LV_ALIGN_CENTER, 0, -40);
    
    // 创建一个按钮
    lv_obj_t *btn = lv_btn_create(lv_scr_act());
    lv_obj_set_size(btn, 120, 50);
    lv_obj_align(btn, LV_ALIGN_CENTER, 0, 0);
    lv_obj_set_style_bg_color(btn, lv_color_hex(0x2196F3), LV_PART_MAIN);
    
    lv_obj_t *btn_label = lv_label_create(btn);
    lv_label_set_text(btn_label, "Click Me");
    lv_obj_center(btn_label);
    
    // 创建一个进度条
    lv_obj_t *bar = lv_bar_create(lv_scr_act());
    lv_obj_set_size(bar, 200, 20);
    lv_obj_align(bar, LV_ALIGN_CENTER, 0, 40);
    lv_bar_set_value(bar, 70, LV_ANIM_OFF);
    
    printf("Demo UI created\n");
}

/*====================
   集成到你的app_example函数
 *====================*/
void app_example_with_lvgl(void)
{
    printf("Starting application with LVGL...\n");
    
    // 1. 初始化你的LCD驱动（保持原有的初始化）
    DisplayLCD_Init();
    printf("LCD display initialized successfully\n");
    
    // 2. 初始化LVGL（使用你的LCD驱动）
    if (lvgl_init_with_your_lcd() != 0) {
        printf("Failed to initialize LVGL\n");
        return;
    }
    
    // 3. 创建演示UI
    create_demo_ui();
    
    // 4. 可以继续添加你的其他初始化代码
    // task_manager_init();
    // task_manager_start_all();
    
    printf("Application started successfully with LVGL\n");
}