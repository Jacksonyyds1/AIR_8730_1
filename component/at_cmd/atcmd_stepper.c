/*
 * 步进电机控制AT命令实现 - 与step_motor.c集成版本
 * 文件: atcmd_stepper.c
 */

#include <stdlib.h>         // 包含atoi函数声明
#include <string.h>         // 包含strlen等字符串函数
#include "atcmd_service.h"
#include "atcmd_stepper.h"

static const char *const TAG = "AT-STEPPER";

// 步进电机AT命令控制状态
static stepper_at_control_t stepper_at_ctrl = {0};
static bool stepper_at_initialized = false;

/**
 * 步进电机AT命令初始化
 */
static int stepper_at_init_hardware(void)
{
    if (stepper_at_initialized) {
        return 0;
    }
    
    // 使用现有的步进电机初始化函数
    stepper_motor_init();
    
    // 初始化AT控制状态
    for (int i = 0; i < MOTOR_COUNT; i++) {
        stepper_at_ctrl.motor_state[i] = STEPPER_AT_STATE_STOPPED;
        stepper_at_ctrl.position[i] = 0;
        stepper_at_ctrl.target_position[i] = 0;
        stepper_at_ctrl.speed[i] = 100;
        stepper_at_ctrl.accel_rate[i] = 100;
    }
    
    stepper_at_initialized = true;
    RTK_LOGI(TAG, "Stepper motor AT command initialized\r\n");
    return 0;
}

/**
 * 获取电机索引名称
 */
static const char* get_motor_name(uint8_t index)
{
    switch (index) {
    case MOTOR_NECK: return "NECK";
    case MOTOR_BASE:   return "BASE";
    default:           return "UNKNOWN";
    }
}

/**
 * 获取电机状态名称
 */
static const char* get_motor_state_name(Motor_State_t state)
{
    switch (state) {
    case Motor_State_Stop:      return "STOPPED";
    case Motor_State_Starting:  return "STARTING";
    case Motor_State_Forward:   return "FORWARD";
    case Motor_State_Backward:  return "BACKWARD";
    case Motor_State_Stopping:  return "STOPPING";
    default:                    return "UNKNOWN";
    }
}

/**
 * 设置电机位置
 */
static int stepper_at_set_position(uint8_t motor_index, int position, uint16_t speed)
{
    if (motor_index >= MOTOR_COUNT) {
        return -1;
    }
    
    if (!stepper_at_initialized) {
        if (stepper_at_init_hardware() != 0) {
            return -1;
        }
    }
    
    // 设置速度配置
    stepper_motor_set_speed_profile(motor_index, speed, stepper_at_ctrl.accel_rate[motor_index]);
    
    // 设置目标位置
    stepper_motor_set_target_position(motor_index, position);
    
    // 更新AT状态
    stepper_at_ctrl.target_position[motor_index] = position;
    stepper_at_ctrl.speed[motor_index] = speed;
    stepper_at_ctrl.motor_state[motor_index] = STEPPER_AT_STATE_MOVING;
    
    RTK_LOGI(TAG, "Motor %s: Moving to position %d at speed %d\r\n", 
             get_motor_name(motor_index), position, speed);
    
    return 0;
}

/**
 * 设置电机方向运动
 */
static int stepper_at_set_direction(uint8_t motor_index, Motor_Direction_t direction, uint16_t speed)
{
    if (motor_index >= MOTOR_COUNT) {
        return -1;
    }
    
    if (!stepper_at_initialized) {
        if (stepper_at_init_hardware() != 0) {
            return -1;
        }
    }
    
    // 设置方向运动
    stepper_motor_set_direction(motor_index, direction, speed);
    
    // 更新AT状态
    stepper_at_ctrl.speed[motor_index] = speed;
    if (direction == Motor_Direction_Stop) {
        stepper_at_ctrl.motor_state[motor_index] = STEPPER_AT_STATE_STOPPING;
    } else {
        stepper_at_ctrl.motor_state[motor_index] = STEPPER_AT_STATE_MOVING;
    }
    
    const char* dir_name = (direction == Motor_Direction_Forward) ? "FORWARD" :
                          (direction == Motor_Direction_Backward) ? "BACKWARD" : "STOP";
    RTK_LOGI(TAG, "Motor %s: Direction %s at speed %d\r\n", 
             get_motor_name(motor_index), dir_name, speed);
    
    return 0;
}

/**
 * 停止电机
 */
static int stepper_at_stop_motor(uint8_t motor_index, bool brake)
{
    if (motor_index >= MOTOR_COUNT) {
        return -1;
    }
    
    if (!stepper_at_initialized) {
        return -1;
    }
    
    stepper_motor_stop(motor_index, brake,0);
    stepper_at_ctrl.motor_state[motor_index] = STEPPER_AT_STATE_STOPPED;
    
    RTK_LOGI(TAG, "Motor %s: Stopped with %s\r\n", 
             get_motor_name(motor_index), brake ? "brake" : "free");
    
    return 0;
}

/**
 * 设置电机当前位置（不移动）
 */
static int stepper_at_reset_position(uint8_t motor_index, int position)
{
    if (motor_index >= MOTOR_COUNT) {
        return -1;
    }
    
    if (!stepper_at_initialized) {
        if (stepper_at_init_hardware() != 0) {
            return -1;
        }
    }
    
    stepper_motor_set_position(motor_index, position);
    stepper_at_ctrl.position[motor_index] = position;
    stepper_at_ctrl.target_position[motor_index] = position;
    
    RTK_LOGI(TAG, "Motor %s: Position reset to %d\r\n", 
             get_motor_name(motor_index), position);
    
    return 0;
}

/**
 * 同步控制两个电机
 */
static int stepper_at_sync_move(int nozzle_target, int base_target, uint16_t speed)
{
    if (!stepper_at_initialized) {
        if (stepper_at_init_hardware() != 0) {
            return -1;
        }
    }
    
    stepper_motor_set_sync_target(nozzle_target, base_target, speed);
    
    stepper_at_ctrl.target_position[MOTOR_NECK] = nozzle_target;
    stepper_at_ctrl.target_position[MOTOR_BASE] = base_target;
    stepper_at_ctrl.speed[MOTOR_NECK] = speed;
    stepper_at_ctrl.speed[MOTOR_BASE] = speed;
    stepper_at_ctrl.motor_state[MOTOR_NECK] = STEPPER_AT_STATE_MOVING;
    stepper_at_ctrl.motor_state[MOTOR_BASE] = STEPPER_AT_STATE_MOVING;
    
    RTK_LOGI(TAG, "Sync move: NECK->%d, BASE->%d at speed %d\r\n", 
             nozzle_target, base_target, speed);
    
    return 0;
}

/**
 * 获取电机状态
 */
static void stepper_at_get_status(void)
{
    if (!stepper_at_initialized) {
        at_printf("Stepper motors not initialized\r\n");
        return;
    }
    
    at_printf("Stepper Motor Status:\r\n");
    
    for (int i = 0; i < MOTOR_COUNT; i++) {
        Motor_State_t hw_state = stepper_motor_get_state(i);
        int current_pos = stepper_motor_get_position(i);
        Motor_Running_Type_t run_type = stepper_motor_get_running_type(i);
        
        at_printf("  Motor %s:\r\n", get_motor_name(i));
        at_printf("    Hardware State: %s\r\n", get_motor_state_name(hw_state));
        at_printf("    Running Type: %s\r\n", (run_type == Motor_Running_Type_Positioning) ? "POSITIONING" : "DIRECTION");
        at_printf("    Current Position: %d\r\n", current_pos);
        at_printf("    Target Position: %d\r\n", stepper_at_ctrl.target_position[i]);
        at_printf("    Speed Setting: %d\r\n", stepper_at_ctrl.speed[i]);
        at_printf("    Accel Rate: %d\r\n", stepper_at_ctrl.accel_rate[i]);
    }
    
    at_printf("  System Status:\r\n");
    at_printf("    All Stopped: %s\r\n", stepper_motor_all_stopped() ? "YES" : "NO");
    at_printf("    Sync Complete: %s\r\n", stepper_motor_is_sync_complete() ? "YES" : "NO");
}

/**
 * AT+STEPPER 命令帮助信息
 */
static void at_stepper_help(void)
{
    RTK_LOGI(TAG, "\r\n");
    RTK_LOGI(TAG, "AT+STEPPER=<operation>[,<param1>][,<param2>][,<param3>]\r\n");
    RTK_LOGI(TAG, "\t<operation>:\r\n");
    RTK_LOGI(TAG, "\t\t0 = GET_STATUS\r\n");
    RTK_LOGI(TAG, "\t\t1 = MOVE <motor_id>,<position>[,<speed>]\r\n");
    RTK_LOGI(TAG, "\t\t2 = DIRECTION <motor_id>,<direction>[,<speed>]\r\n");
    RTK_LOGI(TAG, "\t\t3 = STOP <motor_id>[,<brake>]\r\n");
    RTK_LOGI(TAG, "\t\t4 = RESET_POS <motor_id>,<position>\r\n");
    RTK_LOGI(TAG, "\t\t5 = SYNC_MOVE <nozzle_pos>,<base_pos>[,<speed>]\r\n");
    RTK_LOGI(TAG, "\t\t6 = SET_ACCEL <motor_id>,<rate>\r\n");
    RTK_LOGI(TAG, "\t<param1-3>:\r\n");
    RTK_LOGI(TAG, "\t\tmotor_id: 0=NOZZLE, 1=BASE\r\n");
    RTK_LOGI(TAG, "\t\tposition: -32768 to 32767\r\n");
    RTK_LOGI(TAG, "\t\tdirection: 0=FORWARD, 1=BACKWARD, 2=STOP\r\n");
    RTK_LOGI(TAG, "\t\tspeed: 10-1000 (default 100)\r\n");
    RTK_LOGI(TAG, "\t\tbrake: 0=free, 1=brake (default 1)\r\n");
    RTK_LOGI(TAG, "\t\trate: 1-255 (acceleration rate)\r\n");
    RTK_LOGI(TAG, "Examples:\r\n");
    RTK_LOGI(TAG, "\tAT+STEPPER=0\t\t\t# Get status\r\n");
    RTK_LOGI(TAG, "\tAT+STEPPER=1,0,1000\t\t# Move NOZZLE to pos 1000\r\n");
    RTK_LOGI(TAG, "\tAT+STEPPER=1,1,-500,200\t\t# Move BASE to pos -500, speed 200\r\n");
    RTK_LOGI(TAG, "\tAT+STEPPER=2,0,0,150\t\t# NOZZLE forward, speed 150\r\n");
    RTK_LOGI(TAG, "\tAT+STEPPER=3,1,0\t\t\t# Stop BASE with brake\r\n");
    RTK_LOGI(TAG, "\tAT+STEPPER=4,0,0\t\t\t# Reset NOZZLE position to 0\r\n");
    RTK_LOGI(TAG, "\tAT+STEPPER=5,800,1200,300\t# Sync move both motors\r\n");
}

/**
 * AT+STEPPER 命令处理函数
 */
void at_stepper(void *arg)
{
    int argc = 0;
    char *argv[MAX_ARGC] = {0};
    int operation = -1;
    int param1 = -1, param2 = -1, param3 = -1;
    int error_no = 0;
    
    if (arg == NULL) {
        RTK_LOGW(TAG, "Missing parameters\r\n");
        error_no = 1;
        goto end;
    }
    
    // 解析参数
    argc = parse_param(arg, argv);
    if (argc < 2 || argc > 5) {
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
        param1 = atoi(argv[2]);
    }
    if (argc >= 4 && strlen(argv[3]) > 0) {
        param2 = atoi(argv[3]);
    }
    if (argc >= 5 && strlen(argv[4]) > 0) {
        param3 = atoi(argv[4]);
    }
    
    // 执行相应操作
    switch (operation) {
    case STEPPER_AT_OP_GET_STATUS:
        stepper_at_get_status();
        break;
        
    case STEPPER_AT_OP_MOVE:
        if (param1 < 0 || param1 >= MOTOR_COUNT) {
            RTK_LOGW(TAG, "Invalid motor ID: %d (must be 0-%d)\r\n", param1, MOTOR_COUNT-1);
            error_no = 2;
            break;
        }
        if (argc < 4) {
            RTK_LOGW(TAG, "Position parameter required for MOVE\r\n");
            error_no = 1;
            break;
        }
        uint16_t move_speed = (param3 > 0) ? param3 : 100;  // 默认速度100
        if (stepper_at_set_position(param1, param2, move_speed) != 0) {
            RTK_LOGE(TAG, "Failed to move motor\r\n");
            error_no = 3;
        }
        break;
        
    case STEPPER_AT_OP_DIRECTION:
        if (param1 < 0 || param1 >= MOTOR_COUNT) {
            RTK_LOGW(TAG, "Invalid motor ID: %d\r\n", param1);
            error_no = 2;
            break;
        }
        if (argc < 4) {
            RTK_LOGW(TAG, "Direction parameter required\r\n");
            error_no = 1;
            break;
        }
        if (param2 < 0 || param2 > 2) {
            RTK_LOGW(TAG, "Invalid direction: %d (0=FORWARD, 1=BACKWARD, 2=STOP)\r\n", param2);
            error_no = 2;
            break;
        }
        uint16_t dir_speed = (param3 > 0) ? param3 : 100;  // 默认速度100
        if (stepper_at_set_direction(param1, (Motor_Direction_t)param2, dir_speed) != 0) {
            RTK_LOGE(TAG, "Failed to set direction\r\n");
            error_no = 3;
        }
        break;
        
    case STEPPER_AT_OP_STOP:
        if (param1 < 0 || param1 >= MOTOR_COUNT) {
            RTK_LOGW(TAG, "Invalid motor ID: %d\r\n", param1);
            error_no = 2;
            break;
        }
        bool brake = (param2 != 0);  // 默认使用刹车
        if (stepper_at_stop_motor(param1, brake) != 0) {
            RTK_LOGE(TAG, "Failed to stop motor\r\n");
            error_no = 3;
        }
        break;
        
    case STEPPER_AT_OP_RESET_POS:
        if (param1 < 0 || param1 >= MOTOR_COUNT) {
            RTK_LOGW(TAG, "Invalid motor ID: %d\r\n", param1);
            error_no = 2;
            break;
        }
        if (argc < 4) {
            RTK_LOGW(TAG, "Position parameter required for RESET_POS\r\n");
            error_no = 1;
            break;
        }
        if (stepper_at_reset_position(param1, param2) != 0) {
            RTK_LOGE(TAG, "Failed to reset position\r\n");
            error_no = 3;
        }
        break;
        
    case STEPPER_AT_OP_SYNC_MOVE:
        if (argc < 4) {
            RTK_LOGW(TAG, "Both positions required for SYNC_MOVE\r\n");
            error_no = 1;
            break;
        }
        uint16_t sync_speed = (param3 > 0) ? param3 : 100;  // 默认速度100
        if (stepper_at_sync_move(param1, param2, sync_speed) != 0) {
            RTK_LOGE(TAG, "Failed to sync move\r\n");
            error_no = 3;
        }
        break;
        
    case STEPPER_AT_OP_SET_ACCEL:
        if (param1 < 0 || param1 >= MOTOR_COUNT) {
            RTK_LOGW(TAG, "Invalid motor ID: %d\r\n", param1);
            error_no = 2;
            break;
        }
        if (argc < 4) {
            RTK_LOGW(TAG, "Acceleration rate required\r\n");
            error_no = 1;
            break;
        }
        if (param2 < 1 || param2 > 255) {
            RTK_LOGW(TAG, "Invalid acceleration rate: %d (must be 1-255)\r\n", param2);
            error_no = 2;
            break;
        }
        stepper_at_ctrl.accel_rate[param1] = param2;
        RTK_LOGI(TAG, "Motor %s: Acceleration rate set to %d\r\n", 
                 get_motor_name(param1), param2);
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
            at_stepper_help();
        }
    }
}


// AT命令表
log_item_t at_stepper_items[] = {
    {"+STEPPER", at_stepper, {NULL, NULL}},
};

void print_stepper_at(void)
{
    int i, cmd_len;
    
    cmd_len = sizeof(at_stepper_items) / sizeof(at_stepper_items[0]);
    for (i = 0; i < cmd_len; i++) {
        at_printf("AT%s\r\n", at_stepper_items[i].log_cmd);
    }
}

void at_stepper_init(void)
{
    // 注册AT命令到系统
    atcmd_service_add_table(at_stepper_items, sizeof(at_stepper_items) / sizeof(at_stepper_items[0]));
    RTK_LOGI(TAG, "Stepper motor AT commands initialized\r\n");
}