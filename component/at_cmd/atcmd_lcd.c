/*
 * LCD控制AT命令实现 - 与Lcd.c和lvg_lcd_adapter.c集成版本
 * 文件: atcmd_lcd.c
 */

#include <stdlib.h>         // 包含atoi函数声明
#include <string.h>         // 包含strlen等字符串函数
#include "atcmd_service.h"
#include "atcmd_lcd.h"
#include "Lcd.h"            // 引入现有的LCD控制头文件
#include "lvg_lcd_adapter.h" // 引入LVGL适配器头文件
#include "lvgl.h"

static const char *const TAG = "AT-LCD";

// LCD AT命令控制状态
static lcd_at_control_t lcd_at_ctrl = {0};
static bool lcd_at_initialized = false;

/**
 * LCD AT命令初始化
 */
static int lcd_at_init_hardware(void)
{
    if (lcd_at_initialized) {
        return 0;
    }
    
    // 使用现有的LCD初始化函数
    DisplayLCD_Init();
    
    // 初始化AT控制状态
    lcd_at_ctrl.backlight_on = true;
    lcd_at_ctrl.display_on = true;
    lcd_at_ctrl.brightness = 100;
    lcd_at_ctrl.lvgl_enabled = false;
    
    lcd_at_initialized = true;
    RTK_LOGI(TAG, "LCD AT command initialized\r\n");
    return 0;
}

/**
 * 控制背光开关
 */
static int lcd_at_set_backlight(bool enable)
{
    if (!lcd_at_initialized) {
        if (lcd_at_init_hardware() != 0) {
            return -1;
        }
    }
    
    LCD_BacklightOnOff(enable ? 1 : 0);
    lcd_at_ctrl.backlight_on = enable;
    
    RTK_LOGI(TAG, "LCD backlight %s\r\n", enable ? "ON" : "OFF");
    return 0;
}

/**
 * 填充屏幕颜色
 */
static int lcd_at_fill_screen(uint16_t color)
{
    if (!lcd_at_initialized) {
        if (lcd_at_init_hardware() != 0) {
            return -1;
        }
    }
    
    LCD_Fill_FixedColor_Simple(0, LCD_W-1, 0, LCD_H-1, color);
    
    RTK_LOGI(TAG, "LCD screen filled with color 0x%04X\r\n", color);
    return 0;
}

/**
 * 填充矩形区域
 */
static int lcd_at_fill_rect(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color)
{
    if (!lcd_at_initialized) {
        if (lcd_at_init_hardware() != 0) {
            return -1;
        }
    }
    
    // 参数检查
    if (x1 >= LCD_W || y1 >= LCD_H || x2 >= LCD_W || y2 >= LCD_H || x1 > x2 || y1 > y2) {
        return -1;
    }
    
    LCD_Fill_FixedColor_Simple(x1, x2, y1, y2, color);
    
    RTK_LOGI(TAG, "LCD rect filled (%d,%d) to (%d,%d) with color 0x%04X\r\n", 
             x1, y1, x2, y2, color);
    return 0;
}

/**
 * 使用DMA填充区域
 */
static int lcd_at_fill_area_dma(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color)
{
    if (!lcd_at_initialized) {
        if (lcd_at_init_hardware() != 0) {
            return -1;
        }
    }
    
    // 参数检查
    if (x1 >= LCD_W || y1 >= LCD_H || x2 >= LCD_W || y2 >= LCD_H || x1 > x2 || y1 > y2) {
        return -1;
    }
    
    LCD_Fill_Area_DMA(x1, y1, x2, y2, color);
    
    RTK_LOGI(TAG, "LCD DMA fill (%d,%d) to (%d,%d) with color 0x%04X\r\n", 
             x1, y1, x2, y2, color);
    return 0;
}

/**
 * 启用/禁用LVGL
 */
static int lcd_at_set_lvgl(bool enable)
{
    if (!lcd_at_initialized) {
        if (lcd_at_init_hardware() != 0) {
            return -1;
        }
    }
    
    if (enable && !lcd_at_ctrl.lvgl_enabled) {
        // 启用LVGL
        if (lvgl_init_with_your_lcd() == 0) {
            lcd_at_ctrl.lvgl_enabled = true;
            RTK_LOGI(TAG, "LVGL enabled successfully\r\n");
        } else {
            RTK_LOGE(TAG, "Failed to enable LVGL\r\n");
            return -1;
        }
    } else if (!enable && lcd_at_ctrl.lvgl_enabled) {
        // 1. 停止LVGL任务（需要添加到头文件中）
        lvgl_stop_task();
        
        // 2. 清理所有动画
        lv_anim_del_all();
        
        // 3. 清理屏幕对象
        lv_obj_clean(lv_scr_act());
        
        // 4. 最后一次处理任务
        lv_timer_handler();
        
        // 5. 清空显示
        LCD_Fill_FixedColor_Simple(0, LCD_W-1, 0, LCD_H-1, BLACK);
        
        lcd_at_ctrl.lvgl_enabled = false;
        RTK_LOGI(TAG, "LVGL disabled completely\r\n");
    }
    
    return 0;
}

/**
 * 创建LVGL演示
 */
static int lcd_at_create_lvgl_demo(uint8_t demo_type)
{
    if (!lcd_at_initialized) {
        if (lcd_at_init_hardware() != 0) {
            return -1;
        }
    }
    
    // 确保LVGL已启用
    if (!lcd_at_ctrl.lvgl_enabled) {
        if (lcd_at_set_lvgl(true) != 0) {
            return -1;
        }
    }
    
    switch (demo_type) {
    case LCD_DEMO_LABEL:
        lvgl_demo_label();
        RTK_LOGI(TAG, "LVGL label demo created\r\n");
        break;
        
    case LCD_DEMO_ANIMATION:
        create_animation_demo();
        RTK_LOGI(TAG, "LVGL animation demo created\r\n");
        break;
        
    default:
        RTK_LOGW(TAG, "Unknown demo type: %d\r\n", demo_type);
        return -1;
    }
    
    return 0;
}

/**
 * 获取颜色名称
 */
static const char* get_color_name(uint16_t color)
{
    switch (color) {
    case WHITE:      return "WHITE";
    case BLACK:      return "BLACK";
    case RED:        return "RED";
    case GREEN:      return "GREEN";
    case BLUE:       return "BLUE";
    case YELLOW:     return "YELLOW";
    case CYAN:       return "CYAN";
    case MAGENTA:    return "MAGENTA";
    case ORANGE:     return "ORANGE";
    case PURPLE:     return "PURPLE";
    case PINK:       return "PINK";
    case BROWN:      return "BROWN";
    case GRAY:       return "GRAY";
    default:         return "CUSTOM";
    }
}

/**
 * 获取LCD状态
 */
static void lcd_at_get_status(void)
{
    if (!lcd_at_initialized) {
        at_printf("LCD not initialized\r\n");
        return;
    }
    
    at_printf("LCD Status:\r\n");
    at_printf("  Display: %s\r\n", lcd_at_ctrl.display_on ? "ON" : "OFF");
    at_printf("  Backlight: %s\r\n", lcd_at_ctrl.backlight_on ? "ON" : "OFF");
    at_printf("  Brightness: %d%%\r\n", lcd_at_ctrl.brightness);
    at_printf("  LVGL: %s\r\n", lcd_at_ctrl.lvgl_enabled ? "ENABLED" : "DISABLED");
    at_printf("  Resolution: %dx%d\r\n", LCD_W, LCD_H);
    at_printf("  Controller: ST7789V2\r\n");
    at_printf("  Interface: SPI with DMA\r\n");
}

/**
 * AT+LCD 命令帮助信息
 */
static void at_lcd_help(void)
{
    RTK_LOGI(TAG, "\r\n");
    RTK_LOGI(TAG, "AT+LCD=<operation>[,<param1>][,<param2>][,<param3>][,<param4>]\r\n");
    RTK_LOGI(TAG, "\t<operation>:\r\n");
    RTK_LOGI(TAG, "\t\t0 = GET_STATUS\r\n");
    RTK_LOGI(TAG, "\t\t1 = BACKLIGHT <enable>\r\n");
    RTK_LOGI(TAG, "\t\t2 = FILL_SCREEN <color>\r\n");
    RTK_LOGI(TAG, "\t\t3 = FILL_RECT <x1>,<y1>,<x2>,<y2>,<color>\r\n");
    RTK_LOGI(TAG, "\t\t4 = FILL_DMA <x1>,<y1>,<x2>,<y2>,<color>\r\n");
    RTK_LOGI(TAG, "\t\t5 = LVGL_ENABLE <enable>\r\n");
    RTK_LOGI(TAG, "\t\t6 = LVGL_DEMO <demo_type>\r\n");
    RTK_LOGI(TAG, "\t\t7 = RESET\r\n");
    RTK_LOGI(TAG, "\t<parameters>:\r\n");
    RTK_LOGI(TAG, "\t\tenable: 0=disable, 1=enable\r\n");
    RTK_LOGI(TAG, "\t\tcolor: 0x0000-0xFFFF (RGB565) or predefined:\r\n");
    RTK_LOGI(TAG, "\t\t  WHITE=0xFFFF, BLACK=0x0000, RED=0xF800\r\n");
    RTK_LOGI(TAG, "\t\t  GREEN=0x0721, BLUE=0x001F, YELLOW=0xFEE0\r\n");
    RTK_LOGI(TAG, "\t\tx1,y1,x2,y2: coordinates (0-%d, 0-%d)\r\n", LCD_W-1, LCD_H-1);
    RTK_LOGI(TAG, "\t\tdemo_type: 0=label, 1=animation\r\n");
    RTK_LOGI(TAG, "Examples:\r\n");
    RTK_LOGI(TAG, "\tAT+LCD=0\t\t\t# Get status\r\n");
    RTK_LOGI(TAG, "\tAT+LCD=1,1\t\t\t# Turn backlight on\r\n");
    RTK_LOGI(TAG, "\tAT+LCD=2,0xF800\t\t\t# Fill screen red\r\n");
    RTK_LOGI(TAG, "\tAT+LCD=3,10,10,100,100,0x001F\t# Fill blue rect\r\n");
    RTK_LOGI(TAG, "\tAT+LCD=5,1\t\t\t# Enable LVGL\r\n");
    RTK_LOGI(TAG, "\tAT+LCD=6,1\t\t\t# Show animation demo\r\n");
}

/**
 * AT+LCD 命令处理函数
 */
void at_lcd(void *arg)
{
    int argc = 0;
    char *argv[MAX_ARGC] = {0};
    int operation = -1;
    int param1 = -1, param2 = -1, param3 = -1, param4 = -1, param5 = -1;
    int error_no = 0;
    
    if (arg == NULL) {
        RTK_LOGW(TAG, "Missing parameters\r\n");
        error_no = 1;
        goto end;
    }
    
    // 解析参数
    argc = parse_param(arg, argv);
    if (argc < 2 || argc > 7) {
        RTK_LOGW(TAG, "Invalid parameter count\r\n");
        error_no = 1;
        goto end;
    }
    
    // 获取操作类型
    if (strlen(argv[1]) == 0) {
        RTK_LOGW(TAG, "Missing operation parameter\r\n");
        error_no = 1;
        goto end;
    }
    operation = atoi(argv[1]);
    
    // 获取参数（如果有）
    if (argc >= 3 && strlen(argv[2]) > 0) {
        if (strncmp(argv[2], "0x", 2) == 0 || strncmp(argv[2], "0X", 2) == 0) {
            param1 = (int)strtol(argv[2], NULL, 16);  // 解析十六进制
        } else {
            param1 = atoi(argv[2]);
        }
    }
    if (argc >= 4 && strlen(argv[3]) > 0) {
        param2 = atoi(argv[3]);
    }
    if (argc >= 5 && strlen(argv[4]) > 0) {
        param3 = atoi(argv[4]);
    }
    if (argc >= 6 && strlen(argv[5]) > 0) {
        param4 = atoi(argv[5]);
    }
    if (argc >= 7 && strlen(argv[6]) > 0) {
        if (strncmp(argv[6], "0x", 2) == 0 || strncmp(argv[6], "0X", 2) == 0) {
            param5 = (int)strtol(argv[6], NULL, 16);  // 解析十六进制
        } else {
            param5 = atoi(argv[6]);
        }
    }
    
    // 执行相应操作
    switch (operation) {
    case LCD_AT_OP_GET_STATUS:
        lcd_at_get_status();
        break;
        
    case LCD_AT_OP_BACKLIGHT:
        if (param1 < 0) {
            RTK_LOGW(TAG, "Backlight parameter required (0 or 1)\r\n");
            error_no = 1;
            break;
        }
        if (lcd_at_set_backlight(param1 != 0) != 0) {
            RTK_LOGE(TAG, "Failed to set backlight\r\n");
            error_no = 3;
        }
        break;
        
    case LCD_AT_OP_FILL_SCREEN:
        if (param1 < 0) {
            RTK_LOGW(TAG, "Color parameter required\r\n");
            error_no = 1;
            break;
        }
        if (lcd_at_fill_screen((uint16_t)param1) != 0) {
            RTK_LOGE(TAG, "Failed to fill screen\r\n");
            error_no = 3;
        } else {
            at_printf("Screen filled with %s (0x%04X)\r\n", 
                      get_color_name((uint16_t)param1), (uint16_t)param1);
        }
        break;
        
    case LCD_AT_OP_FILL_RECT:
        if (argc < 7) {
            RTK_LOGW(TAG, "Rectangle parameters required: x1,y1,x2,y2,color\r\n");
            error_no = 1;
            break;
        }
        if (lcd_at_fill_rect((uint16_t)param1, (uint16_t)param2, 
                            (uint16_t)param3, (uint16_t)param4, (uint16_t)param5) != 0) {
            RTK_LOGE(TAG, "Failed to fill rectangle\r\n");
            error_no = 3;
        } else {
            at_printf("Rectangle (%d,%d)-(%d,%d) filled with %s (0x%04X)\r\n", 
                      param1, param2, param3, param4, 
                      get_color_name((uint16_t)param5), (uint16_t)param5);
        }
        break;
        
    case LCD_AT_OP_FILL_DMA:
        if (argc < 7) {
            RTK_LOGW(TAG, "DMA fill parameters required: x1,y1,x2,y2,color\r\n");
            error_no = 1;
            break;
        }
        if (lcd_at_fill_area_dma((uint16_t)param1, (uint16_t)param2, 
                                (uint16_t)param3, (uint16_t)param4, (uint16_t)param5) != 0) {
            RTK_LOGE(TAG, "Failed to DMA fill area\r\n");
            error_no = 3;
        } else {
            at_printf("DMA fill (%d,%d)-(%d,%d) with %s (0x%04X)\r\n", 
                      param1, param2, param3, param4, 
                      get_color_name((uint16_t)param5), (uint16_t)param5);
        }
        break;
        
    case LCD_AT_OP_LVGL_ENABLE:
        if (param1 < 0) {
            RTK_LOGW(TAG, "LVGL enable parameter required (0 or 1)\r\n");
            error_no = 1;
            break;
        }
        if (lcd_at_set_lvgl(param1 != 0) != 0) {
            RTK_LOGE(TAG, "Failed to set LVGL state\r\n");
            error_no = 3;
        }
        break;
        
    case LCD_AT_OP_LVGL_DEMO:
        if (param1 < 0) {
            RTK_LOGW(TAG, "Demo type parameter required\r\n");
            error_no = 1;
            break;
        }
        if (param1 > LCD_DEMO_ANIMATION) {
            RTK_LOGW(TAG, "Invalid demo type: %d (0=label, 1=animation)\r\n", param1);
            error_no = 2;
            break;
        }
        if (lcd_at_create_lvgl_demo((uint8_t)param1) != 0) {
            RTK_LOGE(TAG, "Failed to create LVGL demo\r\n");
            error_no = 3;
        }
        break;
        
    case LCD_AT_OP_RESET:
        // 重新初始化LCD
        lcd_at_initialized = false;
        if (lcd_at_init_hardware() != 0) {
            RTK_LOGE(TAG, "Failed to reset LCD\r\n");
            error_no = 3;
        } else {
            RTK_LOGI(TAG, "LCD reset successfully\r\n");
        }
        break;
        
    default:
        RTK_LOGW(TAG, "Invalid operation: %d\r\n", operation);
        error_no = 1;
        break;
    }
    
end:
    if (error_no == 0) {
        at_printf(ATCMD_OK_END_STR);
    } else {
        at_printf(ATCMD_ERROR_END_STR, error_no);
        if (error_no == 1) {
            at_lcd_help();
        }
    }
}

// AT命令表
log_item_t at_lcd_items[] = {
    {"+LCD", at_lcd, {NULL, NULL}},
};

void print_lcd_at(void)
{
    int i, cmd_len;
    
    cmd_len = sizeof(at_lcd_items) / sizeof(at_lcd_items[0]);
    for (i = 0; i < cmd_len; i++) {
        at_printf("AT%s\r\n", at_lcd_items[i].log_cmd);
    }
}

void at_lcd_init(void)
{
    // 注册AT命令到系统
    atcmd_service_add_table(at_lcd_items, sizeof(at_lcd_items) / sizeof(at_lcd_items[0]));
    RTK_LOGI(TAG, "LCD AT commands initialized\r\n");
}