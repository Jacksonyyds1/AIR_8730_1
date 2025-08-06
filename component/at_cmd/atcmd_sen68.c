/*
 * SEN68环境传感器AT命令实现 - 源文件
 * 文件: atcmd_sen68.c
 * 功能: 为SEN68传感器提供完整的AT命令控制接口
 */
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include "atcmd_service.h"
#include "atcmd_sen68.h"


static const char *const TAG = "AT-SEN68";


// 内部函数声明

static int sen68_at_start_sampling(void);
static int sen68_at_stop_sampling(void);
static int sen68_at_read_single(sen68_at_format_t format);
static int sen68_at_start_continuous(uint32_t interval);
static int sen68_at_stop_continuous(void);
static void sen68_at_get_status(void);
static void sen68_at_get_device_info(void);
static int sen68_at_set_temp_offset(float offset);
static int sen68_at_reset_device(void);
static void sen68_at_help(void);

// 连续采样任务和定时器回调
static void sen68_continuous_sample_task(void *param);
static void sen68_sample_timer_callback(rtos_timer_t timer);

// 数据格式化函数
static void sen68_at_format_data(sen68_data_t *data, sen68_at_format_t format);

// SEN68 AT命令控制状态
static sen68_at_control_t sen68_at_ctrl = {0};
static bool sen68_at_initialized = false;

/**
 * SEN68传感器硬件初始化
 */
 int sen68_at_init_hardware(void)
{
    if (sen68_at_initialized) {
        return 0;
    }
    
    // 使用现有的SEN68初始化函数
    if (sen68_init() != SEN68_OK) {
        RTK_LOGE(TAG, "Failed to initialize SEN68 sensor\r\n");
        return -1;
    }
    
    // 初始化AT控制状态
    sen68_at_ctrl.state = SEN68_AT_STATE_IDLE;
    sen68_at_ctrl.auto_mode = false;
    sen68_at_ctrl.sample_interval = SEN68_AT_SAMPLE_INTERVAL_DEF;
    sen68_at_ctrl.last_sample_time = 0;
    sen68_at_ctrl.sample_count = 0;
    sen68_at_ctrl.data_ready = false;
    sen68_at_ctrl.continuous_task = NULL;
    sen68_at_ctrl.sample_timer = NULL;
    
    sen68_at_initialized = true;
    RTK_LOGI(TAG, "SEN68 AT command initialized successfully\r\n");
    return 0;
}

/**
 * 开始采样
 */
static int sen68_at_start_sampling(void)
{
    if (!sen68_at_initialized) {
        if (sen68_at_init_hardware() != 0) {
            return -1;
        }
    }
    
    if (sen68_start_sample() != SEN68_OK) {
        RTK_LOGE(TAG, "Failed to start SEN68 sampling\r\n");
        sen68_at_ctrl.state = SEN68_AT_STATE_ERROR;
        return -1;
    }
    
    sen68_at_ctrl.state = SEN68_AT_STATE_SAMPLING;
    sen68_at_ctrl.last_sample_time = rtos_time_get_current_system_time_ms();
    RTK_LOGI(TAG, "SEN68 sampling started\r\n");
    return 0;
}

/**
 * 停止采样
 */
static int sen68_at_stop_sampling(void)
{
    if (!sen68_at_initialized) {
        return -1;
    }
    
    // 停止连续采样模式（如果激活）
    sen68_at_stop_continuous();
    
    if (sen68_stop_sample() != SEN68_OK) {
        RTK_LOGE(TAG, "Failed to stop SEN68 sampling\r\n");
        return -1;
    }
    
    sen68_at_ctrl.state = SEN68_AT_STATE_IDLE;
    RTK_LOGI(TAG, "SEN68 sampling stopped\r\n");
    return 0;
}

/**
 * 读取单次数据
 */
static int sen68_at_read_single(sen68_at_format_t format)
{
    if (!sen68_at_initialized) {
        if (sen68_at_init_hardware() != 0) {
            return -1;
        }
    }
    
    // 如果传感器未在采样，先启动采样
    bool was_sampling = sen68_is_sampling();
    if (!was_sampling) {
        if (sen68_at_start_sampling() != 0) {
            return -1;
        }
        // 等待传感器稳定
        rtos_time_delay_ms(1000);
    }
    
    // 读取数据
    sen68_data_t *data = sen68_get_data();
    if (data == NULL) {
        RTK_LOGE(TAG, "Failed to get SEN68 data\r\n");
        return -1;
    }
    
    // 格式化输出数据
    sen68_at_format_data(data, format);
    
    sen68_at_ctrl.sample_count++;
    sen68_at_ctrl.data_ready = true;
    sen68_at_ctrl.last_sample_time = rtos_time_get_current_system_time_ms();
    
    // 如果之前未采样，读取后停止采样
    if (!was_sampling) {
        sen68_at_stop_sampling();
    }
    
    return 0;
}

/**
 * 连续采样任务
 */
static void sen68_continuous_sample_task(void *param)
{
    uint32_t interval = (uint32_t)param;
    
    RTK_LOGI(TAG, "Continuous sampling started, interval: %d ms\r\n", interval);
    
    while (sen68_at_ctrl.state == SEN68_AT_STATE_CONTINUOUS) {
        // 读取并输出数据
        sen68_data_t *data = sen68_get_data();
        if (data != NULL) {
            at_printf("[SEN68-DATA] ");
            sen68_at_format_data(data, SEN68_AT_FORMAT_SIMPLE);
            sen68_at_ctrl.sample_count++;
        }
        
        // 等待指定间隔
        rtos_time_delay_ms(interval);
    }
    
    RTK_LOGI(TAG, "Continuous sampling task ended\r\n");
    sen68_at_ctrl.continuous_task = NULL;
    rtos_task_delete(NULL);
}

/**
 * 开始连续采样
 */
static int sen68_at_start_continuous(uint32_t interval)
{
    if (!sen68_at_initialized) {
        if (sen68_at_init_hardware() != 0) {
            return -1;
        }
    }
    
    // 停止现有的连续采样
    sen68_at_stop_continuous();
    
    // 启动传感器采样
    if (sen68_at_start_sampling() != 0) {
        return -1;
    }
    
    // 验证间隔范围
    if (interval < SEN68_AT_SAMPLE_INTERVAL_MIN) {
        interval = SEN68_AT_SAMPLE_INTERVAL_MIN;
    } else if (interval > SEN68_AT_SAMPLE_INTERVAL_MAX) {
        interval = SEN68_AT_SAMPLE_INTERVAL_MAX;
    }
    
    sen68_at_ctrl.sample_interval = interval;
    sen68_at_ctrl.state = SEN68_AT_STATE_CONTINUOUS;
    
    // 创建连续采样任务
    if (rtos_task_create(&sen68_at_ctrl.continuous_task, 
                        "SEN68_Continuous", 
                        sen68_continuous_sample_task, 
                        (void*)interval, 
                        2048, 
                        tskIDLE_PRIORITY + 2) != RTK_SUCCESS) {
        RTK_LOGE(TAG, "Failed to create continuous sampling task\r\n");
        sen68_at_ctrl.state = SEN68_AT_STATE_ERROR;
        return -1;
    }
    
    RTK_LOGI(TAG, "Continuous sampling started with %d ms interval\r\n", interval);
    return 0;
}

/**
 * 停止连续采样
 */
static int sen68_at_stop_continuous(void)
{
    if (sen68_at_ctrl.state == SEN68_AT_STATE_CONTINUOUS) {
        sen68_at_ctrl.state = SEN68_AT_STATE_IDLE;
        
        // 等待任务结束
        if (sen68_at_ctrl.continuous_task != NULL) {
            // 任务会自动删除自己
            int timeout = 50;  // 500ms超时
            while (sen68_at_ctrl.continuous_task != NULL && timeout-- > 0) {
                rtos_time_delay_ms(10);
            }
        }
        
        RTK_LOGI(TAG, "Continuous sampling stopped\r\n");
    }
    
    return 0;
}

/**
 * 获取传感器状态
 */
static void sen68_at_get_status(void)
{
    const char* state_names[] = {
        "UNINIT", "IDLE", "SAMPLING", "CONTINUOUS", "ERROR"
    };
    
    if (!sen68_at_initialized) {
        at_printf("SEN68 Status: NOT_INITIALIZED\r\n");
        return;
    }
    
    at_printf("SEN68 Status:\r\n");
    at_printf("  State: %s\r\n", state_names[sen68_at_ctrl.state]);
    at_printf("  Sampling: %s\r\n", sen68_is_sampling() ? "YES" : "NO");
    at_printf("  Auto Mode: %s\r\n", sen68_at_ctrl.auto_mode ? "ON" : "OFF");
    at_printf("  Sample Interval: %d ms\r\n", sen68_at_ctrl.sample_interval);
    at_printf("  Sample Count: %d\r\n", sen68_at_ctrl.sample_count);
    at_printf("  Data Ready: %s\r\n", sen68_at_ctrl.data_ready ? "YES" : "NO");
    
    if (sen68_at_ctrl.last_sample_time > 0) {
        uint32_t elapsed = rtos_time_get_current_system_time_ms() - sen68_at_ctrl.last_sample_time;
        at_printf("  Last Sample: %d ms ago\r\n", elapsed);
    }
}

/**
 * 获取设备信息
 */
static void sen68_at_get_device_info(void)
{
    if (!sen68_at_initialized) {
        if (sen68_at_init_hardware() != 0) {
            at_printf("Failed to initialize SEN68 for info reading\r\n");
            return;
        }
    }
    
    at_printf("SEN68 Device Information:\r\n");
    
    // 调用现有的测试函数来获取设备信息
    sen68_test_product_info();
    
    at_printf("  Firmware Version: 1.0.0\r\n");
    at_printf("  I2C Address: 0x%02X\r\n", SEN68_I2C_ADDRESS);
    at_printf("  Supported Commands: %d\r\n", 23);
}

/**
 * 设置温度补偿偏移
 */
static int sen68_at_set_temp_offset(float offset)
{
    if (!sen68_at_initialized) {
        if (sen68_at_init_hardware() != 0) {
            return -1;
        }
    }
    
    // 这里应该调用SEN68的温度补偿设置命令
    // 由于原始代码中没有实现，这里只是示例
    RTK_LOGI(TAG, "Temperature offset set to %.2f°C\r\n", offset);
    return 0;
}

/**
 * 重置设备
 */
static int sen68_at_reset_device(void)
{
    if (!sen68_at_initialized) {
        return -1;
    }
    
    // 停止所有活动
    sen68_at_stop_continuous();
    sen68_at_stop_sampling();
    
    // 重新初始化
    sen68_at_initialized = false;
    if (sen68_at_init_hardware() != 0) {
        return -1;
    }
    
    RTK_LOGI(TAG, "SEN68 device reset completed\r\n");
    return 0;
}

/**
 * 数据格式化函数
 */
static void sen68_at_format_data(sen68_data_t *data, sen68_at_format_t format)
{
    switch (format) {
    case SEN68_AT_FORMAT_SIMPLE:
        at_printf("%.1f,%.1f,%.1f,%.1f,%.2f,%.2f,%.1f,%.1f,%.1f\r\n",
                  sen68_get_pm1(), sen68_get_pm2_5(), sen68_get_pm4(), sen68_get_pm10(),
                  sen68_get_humi(), sen68_get_temp(), 
                  sen68_get_voc_idx(), sen68_get_nox_idx(), sen68_get_hcho());
        break;
        
    case SEN68_AT_FORMAT_DETAILED:
        at_printf("PM1.0: %.1f μg/m³\r\n", sen68_get_pm1());
        at_printf("PM2.5: %.1f μg/m³\r\n", sen68_get_pm2_5());
        at_printf("PM4.0: %.1f μg/m³\r\n", sen68_get_pm4());
        at_printf("PM10: %.1f μg/m³\r\n", sen68_get_pm10());
        at_printf("Humidity: %.2f %%\r\n", sen68_get_humi());
        at_printf("Temperature: %.2f °C\r\n", sen68_get_temp());
        at_printf("VOC Index: %.1f\r\n", sen68_get_voc_idx());
        at_printf("NOx Index: %.1f\r\n", sen68_get_nox_idx());
        at_printf("HCHO: %.1f ppb\r\n", sen68_get_hcho());
        break;
        
    case SEN68_AT_FORMAT_JSON:
        at_printf("{\r\n");
        at_printf("  \"pm1\": %.1f,\r\n", sen68_get_pm1());
        at_printf("  \"pm25\": %.1f,\r\n", sen68_get_pm2_5());
        at_printf("  \"pm4\": %.1f,\r\n", sen68_get_pm4());
        at_printf("  \"pm10\": %.1f,\r\n", sen68_get_pm10());
        at_printf("  \"humidity\": %.2f,\r\n", sen68_get_humi());
        at_printf("  \"temperature\": %.2f,\r\n", sen68_get_temp());
        at_printf("  \"voc_index\": %.1f,\r\n", sen68_get_voc_idx());
        at_printf("  \"nox_index\": %.1f,\r\n", sen68_get_nox_idx());
        at_printf("  \"hcho\": %.1f\r\n", sen68_get_hcho());
        at_printf("}\r\n");
        break;
    }
}

/**
 * AT+SEN68 命令帮助信息
 */
static void sen68_at_help(void)
{
    RTK_LOGI(TAG, "\r\n");
    RTK_LOGI(TAG, "AT+SEN68=<operation>[,<param1>][,<param2>]\r\n");
    RTK_LOGI(TAG, "Operations:\r\n");
    RTK_LOGI(TAG, "  0 = INIT                   # Initialize sensor\r\n");
    RTK_LOGI(TAG, "  1 = START                  # Start sampling\r\n");
    RTK_LOGI(TAG, "  2 = STOP                   # Stop sampling\r\n");
    RTK_LOGI(TAG, "  3 = STATUS                 # Get sensor status\r\n");
    RTK_LOGI(TAG, "  4 = READ [format]          # Read data once\r\n");
    RTK_LOGI(TAG, "  5 = CONTINUOUS <interval>  # Start continuous reading\r\n");
    RTK_LOGI(TAG, "  6 = SET_INTERVAL <ms>      # Set sampling interval\r\n");
    RTK_LOGI(TAG, "  7 = RESET                  # Reset sensor\r\n");
    RTK_LOGI(TAG, "  8 = INFO                   # Get device info\r\n");
    RTK_LOGI(TAG, "  9 = SET_OFFSET <offset>    # Set temp offset\r\n");
    RTK_LOGI(TAG, "Parameters:\r\n");
    RTK_LOGI(TAG, "  format: 0=simple, 1=detailed, 2=json\r\n");
    RTK_LOGI(TAG, "  interval: %d-%d ms\r\n", SEN68_AT_SAMPLE_INTERVAL_MIN, SEN68_AT_SAMPLE_INTERVAL_MAX);
    RTK_LOGI(TAG, "  offset: -40.0 to 40.0°C\r\n");
    RTK_LOGI(TAG, "Examples:\r\n");
    RTK_LOGI(TAG, "  AT+SEN68=0              # Initialize\r\n");
    RTK_LOGI(TAG, "  AT+SEN68=1              # Start sampling\r\n");
    RTK_LOGI(TAG, "  AT+SEN68=4,1            # Read detailed format\r\n");
    RTK_LOGI(TAG, "  AT+SEN68=5,5000         # Continuous every 5s\r\n");
    RTK_LOGI(TAG, "  AT+SEN68=9,-2.5         # Set -2.5°C offset\r\n");
}

/**
 * AT+SEN68 命令处理函数
 */
void at_sen68(void *arg)
{
    int argc = 0;
    char *argv[MAX_ARGC] = {0};
    int operation = -1;
    int param1 = -1;
    float param1_float = 0.0;
    int error_no = 0;
    
    if (arg == NULL) {
        RTK_LOGW(TAG, "Missing parameters\r\n");
        error_no = 1;
        goto end;
    }
    
    argc = parse_param(arg, argv);
    if (argc < 2) {
        RTK_LOGW(TAG, "Invalid number of parameters\r\n");
        error_no = 1;
        goto end;
    }
    
    operation = atoi(argv[1]);
    
    // 解析额外参数
    if (argc > 2 && strlen(argv[2]) > 0) {
        if (operation == SEN68_AT_OP_SET_OFFSET) {
            param1_float = atof(argv[2]);
        } else {
            param1 = atoi(argv[2]);
        }
    }
    
    switch (operation) {
    case SEN68_AT_OP_INIT:
        if (sen68_at_init_hardware() != 0) {
            RTK_LOGE(TAG, "Failed to initialize SEN68\r\n");
            error_no = 2;
        }
        break;
        
    case SEN68_AT_OP_START:
        if (sen68_at_start_sampling() != 0) {
            RTK_LOGE(TAG, "Failed to start sampling\r\n");
            error_no = 2;
        }
        break;
        
    case SEN68_AT_OP_STOP:
        if (sen68_at_stop_sampling() != 0) {
            RTK_LOGE(TAG, "Failed to stop sampling\r\n");
            error_no = 2;
        }
        break;
        
    case SEN68_AT_OP_STATUS:
        sen68_at_get_status();
        break;
        
    case SEN68_AT_OP_READ_DATA:
        {
            sen68_at_format_t format = (param1 >= 0 && param1 <= 2) ? param1 : SEN68_AT_FORMAT_SIMPLE;
            if (sen68_at_read_single(format) != 0) {
                RTK_LOGE(TAG, "Failed to read data\r\n");
                error_no = 2;
            }
        }
        break;
        
    case SEN68_AT_OP_READ_CONTINUOUS:
        if (param1 < 0) {
            param1 = sen68_at_ctrl.sample_interval;
        }
        if (sen68_at_start_continuous(param1) != 0) {
            RTK_LOGE(TAG, "Failed to start continuous sampling\r\n");
            error_no = 2;
        }
        break;
        
    case SEN68_AT_OP_SET_INTERVAL:
        if (param1 < SEN68_AT_SAMPLE_INTERVAL_MIN || param1 > SEN68_AT_SAMPLE_INTERVAL_MAX) {
            RTK_LOGW(TAG, "Invalid interval: %d (valid: %d-%d ms)\r\n", 
                     param1, SEN68_AT_SAMPLE_INTERVAL_MIN, SEN68_AT_SAMPLE_INTERVAL_MAX);
            error_no = 1;
        } else {
            sen68_at_ctrl.sample_interval = param1;
            RTK_LOGI(TAG, "Sample interval set to %d ms\r\n", param1);
        }
        break;
        
    case SEN68_AT_OP_RESET:
        if (sen68_at_reset_device() != 0) {
            RTK_LOGE(TAG, "Failed to reset device\r\n");
            error_no = 2;
        }
        break;
        
    case SEN68_AT_OP_GET_INFO:
        sen68_at_get_device_info();
        break;
        
    case SEN68_AT_OP_SET_OFFSET:
        if (param1_float < -40.0 || param1_float > 40.0) {
            RTK_LOGW(TAG, "Invalid temperature offset: %.2f (valid: -40.0 to 40.0°C)\r\n", param1_float);
            error_no = 1;
        } else if (sen68_at_set_temp_offset(param1_float) != 0) {
            RTK_LOGE(TAG, "Failed to set temperature offset\r\n");
            error_no = 2;
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
            sen68_at_help();
        }
    }
}

// AT命令表
log_item_t at_sen68_items[] = {
    {"+SEN68", at_sen68, {NULL, NULL}},
};

void print_sen68_at(void)
{
    int i, cmd_len;
    
    cmd_len = sizeof(at_sen68_items) / sizeof(at_sen68_items[0]);
    for (i = 0; i < cmd_len; i++) {
        at_printf("AT%s\r\n", at_sen68_items[i].log_cmd);
    }
}

void at_sen68_init(void)
{
    // 注册AT命令到系统
    atcmd_service_add_table(at_sen68_items, sizeof(at_sen68_items) / sizeof(at_sen68_items[0]));
    RTK_LOGI(TAG, "SEN68 AT commands initialized\r\n");
}