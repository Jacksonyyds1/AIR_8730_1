/*
 * 风扇控制AT命令实现 - 与Fan_motor.c集成版本
 * 文件: atcmd_fan.c
 */
#include <stdlib.h>
#include <string.h>
#include "atcmd_service.h"
#include "atcmd_fan.h"


static const char *const TAG = "AT-FAN";

// 风扇控制状态
static fan_at_control_t fan_at_ctrl = {0};
static bool fan_at_initialized = false;

/**
 * 风扇AT命令初始化
 */
static int fan_at_init_hardware(void)
{
    if (fan_at_initialized) {
        return 0;
    }
    
    // 使用现有的风扇控制初始化
    // RPM表已经在Fan_motor.c中定义，直接调用
    fan_controller_example();
    
    // 初始化AT控制状态
    fan_at_ctrl.state = FAN_AT_STATE_OFF;
    fan_at_ctrl.speed_level = 0;
    fan_at_ctrl.auto_mode = false;
    fan_at_ctrl.te_mode = false;
    fan_at_ctrl.target_rpm = 0;
    
    fan_at_initialized = true;
    RTK_LOGI(TAG, "Fan AT command initialized with existing motor controller\r\n");
    return 0;
}

/**
 * 设置风扇开关状态
 */
static int fan_at_set_power(fan_at_state_t state)
{
    if (!fan_at_initialized) {
        if (fan_at_init_hardware() != 0) {
            return -1;
        }
    }
    
    switch (state) {
    case FAN_AT_STATE_OFF:
        // 设置为档位0（停转）
        fan_speed_set_auto_mode(false);
        fan_speed_set_speed(0, false);
        fan_at_ctrl.speed_level = 0;
        RTK_LOGI(TAG, "Fan turned OFF\r\n");
        break;
        
    case FAN_AT_STATE_ON:
        // 开启风扇，如果没有设置档位则使用默认档位1
        if (fan_at_ctrl.speed_level == 0) {
            fan_at_ctrl.speed_level = 1;
        }
        fan_speed_set_auto_mode(fan_at_ctrl.auto_mode);
        fan_speed_set_speed(fan_at_ctrl.speed_level, fan_at_ctrl.auto_mode);
        RTK_LOGI(TAG, "Fan turned ON, level: %d\r\n", fan_at_ctrl.speed_level);
        break;
        
    default:
        return -1;
    }
    
    fan_at_ctrl.state = state;
    return 0;
}

/**
 * 设置风扇速度档位（1-5档，0为停转）
 */
static int fan_at_set_speed_level(uint8_t level)
{
    if (level > FAN_MAX_SPEED_LEVEL) {
        return -1;
    }
    
    if (!fan_at_initialized) {
        if (fan_at_init_hardware() != 0) {
            return -1;
        }
    }
    
    fan_at_ctrl.speed_level = level;
    fan_at_ctrl.te_mode = false; // 退出TE模式
    
    // 如果风扇处于开启状态，立即应用新档位
    if (fan_at_ctrl.state == FAN_AT_STATE_ON || level > 0) {
        fan_speed_set_auto_mode(fan_at_ctrl.auto_mode);
        fan_speed_set_speed(level, fan_at_ctrl.auto_mode);
        
        if (level > 0) {
            fan_at_ctrl.state = FAN_AT_STATE_ON;
        } else {
            fan_at_ctrl.state = FAN_AT_STATE_OFF;
        }
        
        RTK_LOGI(TAG, "Fan speed level set to %d\r\n", level);
    }
    
    return 0;
}

/**
 * 设置风扇目标转速（TE模式）
 */
static int fan_at_set_target_rpm(int rpm)
{
    if (rpm < 0 || rpm > FAN_MAX_RPM) {
        return -1;
    }
    
    if (!fan_at_initialized) {
        if (fan_at_init_hardware() != 0) {
            return -1;
        }
    }
    
    fan_at_ctrl.target_rpm = rpm;
    fan_at_ctrl.te_mode = true;
    
    if (rpm > 0) {
        fan_speed_te_set(rpm);
        fan_at_ctrl.state = FAN_AT_STATE_ON;
        RTK_LOGI(TAG, "Fan TE mode set to %d RPM\r\n", rpm);
    } else {
        fan_speed_set_speed(0, false);
        fan_at_ctrl.state = FAN_AT_STATE_OFF;
        fan_at_ctrl.te_mode = false;
        RTK_LOGI(TAG, "Fan TE mode disabled\r\n");
    }
    
    return 0;
}

/**
 * 设置自动模式
 */
static int fan_at_set_auto_mode(bool enable)
{
    if (!fan_at_initialized) {
        if (fan_at_init_hardware() != 0) {
            return -1;
        }
    }
    
    fan_at_ctrl.auto_mode = enable;
    fan_speed_set_auto_mode(enable);
    
    RTK_LOGI(TAG, "Fan auto mode %s\r\n", enable ? "enabled" : "disabled");
    return 0;
}

/**
 * 获取风扇状态
 */
static void fan_at_get_status(void)
{
    if (!fan_at_initialized) {
        at_printf("Fan not initialized\r\n");
        return;
    }
    
    // 获取实时数据
    int current_rpm = fan_speed_get_rpm();
    int current_duty = fan_speed_get_current_duty();
    int fg_pps = fan_speed_get_fg_pps();
    
    at_printf("Fan Status:\r\n");
    at_printf("  State: %s\r\n", (fan_at_ctrl.state == FAN_AT_STATE_ON) ? "ON" : "OFF");
    at_printf("  Speed Level: %d\r\n", fan_at_ctrl.speed_level);
    at_printf("  Auto Mode: %s\r\n", fan_at_ctrl.auto_mode ? "ON" : "OFF");
    at_printf("  TE Mode: %s\r\n", fan_at_ctrl.te_mode ? "ON" : "OFF");
    
    if (fan_at_ctrl.te_mode) {
        at_printf("  Target RPM: %d\r\n", fan_at_ctrl.target_rpm);
    }
    
    at_printf("  Current RPM: %d\r\n", current_rpm);
    at_printf("  PWM Duty: %d/%d\r\n", current_duty, FAN_PWM_MAX_DUTY);
    at_printf("  FG PPS: %d\r\n", fg_pps);
}

/**
 * AT+FAN 命令帮助信息
 */
static void at_fan_help(void)
{
    RTK_LOGI(TAG, "\r\n");
    RTK_LOGI(TAG, "AT+FAN=<operation>[,<param1>][,<param2>]\r\n");
    RTK_LOGI(TAG, "\t<operation>:\r\n");
    RTK_LOGI(TAG, "\t\t0 = POWER_OFF\r\n");
    RTK_LOGI(TAG, "\t\t1 = POWER_ON [,level]\r\n");
    RTK_LOGI(TAG, "\t\t2 = GET_STATUS\r\n");
    RTK_LOGI(TAG, "\t\t3 = SET_LEVEL <level>\r\n");
    RTK_LOGI(TAG, "\t\t4 = SET_RPM <rpm>\r\n");
    RTK_LOGI(TAG, "\t\t5 = SET_AUTO_MODE <enable>\r\n");
    RTK_LOGI(TAG, "\t<param1>:\r\n");
    RTK_LOGI(TAG, "\t\tlevel: 0-5 (0=stop, 1-5=speed levels)\r\n");
    RTK_LOGI(TAG, "\t\trpm: 0-%d (direct RPM control)\r\n", FAN_MAX_RPM);
    RTK_LOGI(TAG, "\t\tenable: 0=disable, 1=enable\r\n");
    RTK_LOGI(TAG, "Examples:\r\n");
    RTK_LOGI(TAG, "\tAT+FAN=0\t\t\t# Turn fan off\r\n");
    RTK_LOGI(TAG, "\tAT+FAN=1\t\t\t# Turn fan on (default level)\r\n");
    RTK_LOGI(TAG, "\tAT+FAN=1,3\t\t\t# Turn fan on, level 3\r\n");
    RTK_LOGI(TAG, "\tAT+FAN=2\t\t\t# Get status\r\n");
    RTK_LOGI(TAG, "\tAT+FAN=3,4\t\t\t# Set to level 4\r\n");
    RTK_LOGI(TAG, "\tAT+FAN=4,2500\t\t# Set to 2500 RPM\r\n");
    RTK_LOGI(TAG, "\tAT+FAN=5,1\t\t\t# Enable auto mode\r\n");
}

/**
 * AT+FAN 命令处理函数
 */
void at_fan(void *arg)
{
    int argc = 0;
    char *argv[MAX_ARGC] = {0};
    int operation = -1;
    int param1 = -1;
    int error_no = 0;
    
    if (arg == NULL) {
        RTK_LOGW(TAG, "Missing parameters\r\n");
        error_no = 1;
        goto end;
    }
    
    // 解析参数
    argc = parse_param(arg, argv);
    if (argc < 2 || argc > 4) {
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
    
    // 获取参数1（如果有）
    if (argc >= 3 && strlen(argv[2]) > 0) {
        param1 = atoi(argv[2]);
    }
    
    // 执行相应操作
    switch (operation) {
    case FAN_AT_OP_POWER_OFF:
        if (fan_at_set_power(FAN_AT_STATE_OFF) != 0) {
            RTK_LOGE(TAG, "Failed to turn off fan\r\n");
            error_no = 3;
        }
        break;
        
    case FAN_AT_OP_POWER_ON:
        // 如果指定了档位参数，先设置档位
        if (param1 >= 0) {
            if (param1 > FAN_MAX_SPEED_LEVEL) {
                RTK_LOGW(TAG, "Speed level must be 0-%d\r\n", FAN_MAX_SPEED_LEVEL);
                error_no = 2;
                break;
            }
            if (fan_at_set_speed_level(param1) != 0) {
                RTK_LOGE(TAG, "Failed to set speed level\r\n");
                error_no = 3;
                break;
            }
        }
        if (fan_at_set_power(FAN_AT_STATE_ON) != 0) {
            RTK_LOGE(TAG, "Failed to turn on fan\r\n");
            error_no = 3;
        }
        break;
        
    case FAN_AT_OP_GET_STATUS:
        fan_at_get_status();
        break;
        
    case FAN_AT_OP_SET_LEVEL:
        if (param1 < 0) {
            RTK_LOGW(TAG, "Level parameter required\r\n");
            error_no = 1;
            break;
        }
        if (param1 > FAN_MAX_SPEED_LEVEL) {
            RTK_LOGW(TAG, "Speed level must be 0-%d\r\n", FAN_MAX_SPEED_LEVEL);
            error_no = 2;
            break;
        }
        if (fan_at_set_speed_level(param1) != 0) {
            RTK_LOGE(TAG, "Failed to set speed level\r\n");
            error_no = 3;
        }
        break;
        
    case FAN_AT_OP_SET_RPM:
        if (param1 < 0) {
            RTK_LOGW(TAG, "RPM parameter required\r\n");
            error_no = 1;
            break;
        }
        if (fan_at_set_target_rpm(param1) != 0) {
            RTK_LOGE(TAG, "Failed to set target RPM\r\n");
            error_no = 3;
        }
        break;
        
    case FAN_AT_OP_SET_AUTO_MODE:
        if (param1 < 0) {
            RTK_LOGW(TAG, "Auto mode parameter required (0 or 1)\r\n");
            error_no = 1;
            break;
        }
        if (fan_at_set_auto_mode(param1 != 0) != 0) {
            RTK_LOGE(TAG, "Failed to set auto mode\r\n");
            error_no = 3;
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
            at_fan_help();
        }
    }
}




// AT命令表
log_item_t at_fan_items[] = {
    {"+FAN", at_fan, {NULL, NULL}},
};

void print_fan_at(void)
{
    int i, cmd_len;
    
    cmd_len = sizeof(at_fan_items) / sizeof(at_fan_items[0]);
    for (i = 0; i < cmd_len; i++) {
        at_printf("AT%s\r\n", at_fan_items[i].log_cmd);
    }
}

void at_fan_init(void)
{
    // 注册AT命令到系统
    atcmd_service_add_table(at_fan_items, sizeof(at_fan_items) / sizeof(at_fan_items[0]));
    RTK_LOGI(TAG, "Fan AT commands initialized with motor controller integration\r\n");
}