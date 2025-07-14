/*
 * LVGL 8.3 Compatible LCD Adapter for RTL8721DCM with ST7789V
 * Optimized for performance and reliability
 */
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "lvgl.h"
#include "os_wrapper.h"

#include "../../component/ui/drivers/st7789v/include/st7789v.h"

// LCD 配置参数 - 针对ST7789V
#define LCD_W               240
#define LCD_H               240
#define LVGL_TICK_PERIOD    5    // 5ms
#define LVGL_TASK_PRIORITY  2
#define LVGL_TASK_STACK     (1024 * 4)

// 显示缓冲区配置 - 使用部分缓冲以节省内存
#define DISP_BUF_SIZE       (LCD_W * 60 * 2)  // 60行缓冲，RGB565格式

static uint8_t g_draw_buf_1[DISP_BUF_SIZE];
static uint8_t g_draw_buf_2[DISP_BUF_SIZE];  // 双缓冲

// LVGL 8.3 驱动结构
static lv_disp_drv_t g_disp_drv;
static lv_disp_draw_buf_t g_disp_buf;
static lv_disp_t *g_display = NULL;

// LVGL任务相关
static rtos_task_t lvgl_task_handle = NULL;
static volatile bool lvgl_running = false;

// 性能统计
typedef struct {
    uint32_t flush_count;
    uint32_t flush_error_count;
    uint32_t last_flush_time;
    uint32_t max_flush_time;
    uint32_t total_pixels_transferred;
} lvgl_perf_stats_t;

static lvgl_perf_stats_t g_perf_stats = {0};

/**
 * LVGL tick 回调函数
 */
uint32_t lvgl_tick_get_cb(void)
{
    return rtos_time_get_current_system_time_ms();
}

/**
 * 显示刷新回调函数
 */
static void lvgl_flush_cb(lv_disp_drv_t *disp_drv, const lv_area_t *area, lv_color_t *color_p)
{
    uint32_t start_time = rtos_time_get_current_system_time_ms();
    
    // 参数有效性检查
    if (area == NULL || color_p == NULL) {
        g_perf_stats.flush_error_count++;
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
    uint32_t pixel_count = width * height;
    
    // 边界检查
    if (x2 >= LCD_W || y2 >= LCD_H) {
        printf("LVGL flush: Invalid area (%d,%d)-(%d,%d)\n", x1, y1, x2, y2);
        g_perf_stats.flush_error_count++;
        lv_disp_flush_ready(disp_drv);
        return;
    }
    
    // 使用st7789v的清理和刷新函数
    st7789v_clean_invalidate_buffer((uint8_t*)color_p);
    
    // 更新性能统计
    g_perf_stats.flush_count++;
    g_perf_stats.total_pixels_transferred += pixel_count;
    
    uint32_t flush_time = rtos_time_get_current_system_time_ms() - start_time;
    g_perf_stats.last_flush_time = flush_time;
    if (flush_time > g_perf_stats.max_flush_time) {
        g_perf_stats.max_flush_time = flush_time;
    }
    
    // 通知LVGL刷新完成
    lv_disp_flush_ready(disp_drv);
}

/**
 * LVGL任务处理函数 - 优化版本
 */
static void lvgl_task_handler(void *param)
{
    (void)param;
    uint32_t last_time = rtos_time_get_current_system_time_ms();
    
    printf("LVGL task started\n");
    
    while (lvgl_running) {
        uint32_t current_time = rtos_time_get_current_system_time_ms();
        
        // 处理LVGL事件和绘制
        uint32_t sleep_time = lv_timer_handler();
        
        // 动态调整延时时间，提高响应性
        if (sleep_time > 100) {
            sleep_time = 100;  // 最大不超过100ms
        }
        if (sleep_time < 5) {
            sleep_time = 5;    // 最小5ms，避免CPU占用过高
        }
        
        rtos_time_delay_ms(sleep_time);
        
        // 每秒打印一次性能统计（调试用）
        if (current_time - last_time >= 10000) {  // 10秒
            printf("LVGL Stats: flushes=%lu, errors=%lu, max_time=%lums, total_pixels=%lu\n",
                   g_perf_stats.flush_count, g_perf_stats.flush_error_count,
                   g_perf_stats.max_flush_time, g_perf_stats.total_pixels_transferred);
            last_time = current_time;
        }
    }
    
    printf("LVGL task stopped\n");
    rtos_task_delete(NULL);
}

/**
 * LVGL VBlank 回调 - 与st7789v驱动集成
 */
static void lvgl_vblank_callback(void *data)
{
    (void)data;
    // 在VBlank期间处理LVGL刷新，减少撕裂
    lv_timer_handler();
}

/**
 * 初始化LVGL和LCD适配器 - 优化版本
 */
int lvgl_init_with_your_lcd(void)
{
    int lcd_width, lcd_height;
    
    printf("=== Initializing LVGL with ST7789V ===\n");
    
    // 1. 初始化ST7789V LCD硬件
    printf("Initializing ST7789V hardware...\n");
    st7789v_init();
    
    // 获取LCD实际尺寸
    st7789v_get_info(&lcd_width, &lcd_height);
    printf("LCD info: %dx%d\n", lcd_width, lcd_height);
    
    // 验证尺寸匹配
    if (lcd_width != LCD_W || lcd_height != LCD_H) {
        printf("Warning: LCD size mismatch. Expected %dx%d, got %dx%d\n",
               LCD_W, LCD_H, lcd_width, lcd_height);
    }
    
    // 2. 初始化LVGL核心
    printf("Initializing LVGL core...\n");
    lv_init();
    
    // 3. 设置tick回调
    lv_tick_set_cb(lvgl_tick_get_cb);
    
    // 4. 初始化绘制缓冲区 (LVGL 8.3 API)
    printf("Initializing display buffer (%d bytes)...\n", DISP_BUF_SIZE);
    lv_disp_draw_buf_init(&g_disp_buf, g_draw_buf_1, g_draw_buf_2, DISP_BUF_SIZE / 2);
    
    // 5. 初始化显示驱动
    printf("Initializing display driver...\n");
    lv_disp_drv_init(&g_disp_drv);
    g_disp_drv.hor_res = LCD_W;
    g_disp_drv.ver_res = LCD_H;
    g_disp_drv.flush_cb = lvgl_flush_cb;
    g_disp_drv.draw_buf = &g_disp_buf;
    
    // 性能优化设置
    g_disp_drv.full_refresh = 0;  // 启用部分刷新
    g_disp_drv.antialiasing = 1;  // 启用抗锯齿
    
    // 6. 注册显示设备
    g_display = lv_disp_drv_register(&g_disp_drv);
    if (g_display == NULL) {
        printf("Failed to register display driver\n");
        return -1;
    }
    
    // 7. 注册VBlank回调以优化刷新时机
    ST7789VVBlankCallback vblank_cb = {
        .VBlank = lvgl_vblank_callback
    };
    st7789v_register_callback(&vblank_cb, NULL);
    
    // 8. 创建LVGL处理任务
    printf("Creating LVGL task...\n");
    lvgl_running = true;
    if (rtos_task_create(&lvgl_task_handle, "lvgl_task", lvgl_task_handler, 
                         NULL, LVGL_TASK_STACK, LVGL_TASK_PRIORITY) != SUCCESS) {
        printf("Failed to create LVGL task\n");
        lvgl_running = false;
        return -1;
    }
    
    printf("=== LVGL initialization completed successfully ===\n");
    return 0;
}

/**
 * 去初始化LVGL
 */
void lvgl_deinit(void)
{
    printf("Deinitializing LVGL...\n");
    
    // 停止任务
    lvgl_running = false;
    if (lvgl_task_handle != NULL) {
        rtos_time_delay_ms(100);  // 等待任务退出
        lvgl_task_handle = NULL;
    }
    
    // 注销显示设备
    if (g_display != NULL) {
        lv_disp_remove(g_display);
        g_display = NULL;
    }
    
    // 清理性能统计
    memset(&g_perf_stats, 0, sizeof(g_perf_stats));
    
    printf("LVGL deinitialized\n");
}

/**
 * 获取性能统计信息
 */
void lvgl_get_performance_stats(lvgl_perf_stats_t *stats)
{
    if (stats != NULL) {
        memcpy(stats, &g_perf_stats, sizeof(lvgl_perf_stats_t));
    }
}

/**
 * 创建基础UI演示
 */
void create_basic_ui(void)
{
    // 设置屏幕背景为深蓝色
    lv_obj_t *screen = lv_scr_act();
    lv_obj_set_style_bg_color(screen, lv_color_hex(0x001122), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(screen, LV_OPA_COVER, LV_PART_MAIN);
    
    // 创建标题标签
    lv_obj_t *title_label = lv_label_create(screen);
    lv_label_set_text(title_label, "LVGL + ST7789V");
    lv_obj_set_style_text_color(title_label, lv_color_hex(0xFFFFFF), LV_PART_MAIN);
    lv_obj_set_style_text_font(title_label, &lv_font_montserrat_18, LV_PART_MAIN);
    lv_obj_align(title_label, LV_ALIGN_TOP_MID, 0, 20);
    
    // 创建状态标签
    lv_obj_t *status_label = lv_label_create(screen);
    lv_label_set_text(status_label, "RTL8721DCM\nReady!");
    lv_obj_set_style_text_color(status_label, lv_color_hex(0x00FF00), LV_PART_MAIN);
    lv_obj_set_style_text_align(status_label, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);
    lv_obj_align(status_label, LV_ALIGN_CENTER, 0, 0);
    
    // 创建版本信息标签
    lv_obj_t *version_label = lv_label_create(screen);
    lv_label_set_text_fmt(version_label, "LVGL v%d.%d.%d", 
                          lv_version_major(), lv_version_minor(), lv_version_patch());
    lv_obj_set_style_text_color(version_label, lv_color_hex(0xCCCCCC), LV_PART_MAIN);
    lv_obj_align(version_label, LV_ALIGN_BOTTOM_MID, 0, -20);
    
    printf("Basic UI created\n");
}

/**
 * 创建动画演示
 */
static void anim_x_cb(void *obj, int32_t value)
{
    lv_obj_set_x((lv_obj_t*)obj, (lv_coord_t)value);
}

void create_animation_demo(void)
{
    // 设置屏幕背景
    lv_obj_t *screen = lv_scr_act();
    lv_obj_set_style_bg_color(screen, lv_color_hex(0x000000), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(screen, LV_OPA_COVER, LV_PART_MAIN);
    
    // 创建动画方块
    lv_obj_t *box = lv_obj_create(screen);
    lv_obj_set_size(box, 50, 50);
    lv_obj_set_style_bg_color(box, lv_color_hex(0xFF6600), LV_PART_MAIN);
    lv_obj_set_style_border_width(box, 3, LV_PART_MAIN);
    lv_obj_set_style_border_color(box, lv_color_hex(0xFFFFFF), LV_PART_MAIN);
    lv_obj_set_style_radius(box, 10, LV_PART_MAIN);
    lv_obj_set_pos(box, 10, LCD_H / 2 - 25);
    
    // 创建平滑动画
    lv_anim_t a;
    lv_anim_init(&a);
    lv_anim_set_var(&a, box);
    lv_anim_set_values(&a, 10, LCD_W - 60);
    lv_anim_set_time(&a, 2000);  // 2秒
    lv_anim_set_exec_cb(&a, anim_x_cb);
    lv_anim_set_path_cb(&a, lv_anim_path_ease_in_out);
    lv_anim_set_repeat_count(&a, LV_ANIM_REPEAT_INFINITE);
    lv_anim_set_playback_time(&a, 2000);
    lv_anim_start(&a);
    
    printf("Animation demo created\n");
}

/**
 * 测试基础显示功能
 */
void test_basic_display(void)
{
    printf("Testing basic display functionality...\n");
    
    // 清屏测试
    lv_obj_t *screen = lv_scr_act();
    lv_obj_set_style_bg_color(screen, lv_color_hex(0xFF0000), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(screen, LV_OPA_COVER, LV_PART_MAIN);
    rtos_time_delay_ms(500);
    
    lv_obj_set_style_bg_color(screen, lv_color_hex(0x00FF00), LV_PART_MAIN);
    rtos_time_delay_ms(500);
    
    lv_obj_set_style_bg_color(screen, lv_color_hex(0x0000FF), LV_PART_MAIN);
    rtos_time_delay_ms(500);
    
    lv_obj_set_style_bg_color(screen, lv_color_hex(0x000000), LV_PART_MAIN);
    
    printf("Basic display test completed\n");
}