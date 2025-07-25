#include "ameba_soc.h"
#include "absolute_encoder.h"  
#include "step_motor.h"


#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"

#include "encoder.h"

/*
 * 新的运行时校准接口使用示例：
 * 
 * 1. 初始化时自动配置和启动校准：
 *    encoder_init();  // 已在初始化中配置运行时校准
 * 
 * 2. 在主循环中监控校准状态：
 *    encoder_monitor_calibration_status();
 * 
 * 3. 手动控制校准过程：
 *    encoder_start_runtime_calibration(MOTOR_NECK);     // 启动校准
 *    encoder_force_calibration_analysis(MOTOR_NECK);    // 强制分析
 *    encoder_reset_runtime_calibration(MOTOR_NECK);     // 重置校准
 * 
 * 4. 查询校准结果：
 *    runtime_calibration_result_t result;
 *    encoder_get_runtime_calibration_status(MOTOR_NECK, &result);
 *    if(result.state == RUNTIME_CALIBRATION_CALIBRATED) {
 *        printf("校准完成：步数=%d，置信度=%.2f\n", 
 *               result.calibrated_steps_per_unit, result.confidence_level);
 *    }
 * 
 * 5. 启用/禁用动态校准：
 *    encoder_set_dynamic_calibration(MOTOR_NECK, true);  // 启用
 *    encoder_set_dynamic_calibration(MOTOR_NECK, false); // 禁用
 */

const uint8_t base_encoder_map_data[350] =   // k=9的德布鲁因序列，裁剪至350位，从任意位置出发最长可在11位内定位
{
    1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 0, 0, 1, 1, 1, 1, 1, 1, 0, 1, 0, 1, 1, 1, 1, 1, 1, 0, 0, 0, 1, 1, 1, 1, 1, 0, 1, 1, 0, 1, 1, 1, 1,
    1, 0, 1, 0, 0, 1, 1, 1, 1, 1, 0, 0, 1, 0, 1, 1, 1, 1, 1, 0, 0, 0, 0, 1, 1, 1, 1, 0, 1, 1, 1, 0, 1, 1, 1, 1, 0, 1, 1, 0, 0, 1, 1, 1, 1, 0, 1, 0, 1, 0,
    1, 1, 1, 1, 0, 1, 0, 0, 0, 1, 1, 1, 1, 0, 0, 1, 1, 0, 1, 1, 1, 1, 0, 0, 1, 0, 0, 1, 1, 1, 1, 0, 0, 0, 1, 0, 1, 1, 1, 1, 0, 0, 0, 0, 0, 1, 1, 1, 0, 1,
    1, 1, 0, 0, 1, 1, 1, 0, 1, 1, 0, 1, 0, 1, 1, 1, 0, 1, 1, 0, 0, 0, 1, 1, 1, 0, 1, 0, 1, 1, 0, 1, 1, 1, 0, 1, 0, 1, 0, 0, 1, 1, 1, 0, 1, 0, 0, 1, 0, 1,
    1, 1, 0, 1, 0, 0, 0, 0, 1, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 1, 0, 0, 1, 0, 1, 0, 1, 1, 1, 0, 0, 1, 0, 0, 0, 1, 1, 1, 0, 0, 0, 1, 1, 0, 1, 1, 1, 0, 0, 0,
    1, 0, 0, 1, 1, 1, 0, 0, 0, 0, 1, 0, 1, 1, 1, 0, 0, 0, 0, 0, 0, 1, 1, 0, 1, 1, 0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0, 1, 1, 0, 0, 0,
    0, 1, 1, 0, 1, 0, 1, 1, 0, 0, 1, 1, 0, 1, 0, 1, 0, 1, 0, 1, 1, 0, 1, 0, 1, 0, 0, 0, 1, 1, 0, 1, 0, 0, 1, 0, 0, 1, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0,
};

const uint8_t neck_encoder_map_data[180] =   // k=8的德布鲁因序列，裁剪至180位，从任意位置出发最长可在9位内定位
{
    1, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 0, 0, 1, 1, 1, 1, 1, 0, 1, 0, 1, 1, 1, 1, 1, 0, 0, 0, 1, 1, 1, 1, 0, 1, 1, 0, 1, 1, 1, 1, 0, 1, 0, 0, 1,
    1, 1, 1, 0, 0, 1, 0, 1, 1, 1, 1, 0, 0, 0, 0, 1, 1, 1, 0, 1, 1, 1, 0, 1, 1, 0, 0, 1, 1, 1, 0, 1, 0, 1, 0, 1, 1, 1, 0, 1, 0, 0, 0, 1, 1, 1, 0, 0, 1, 1,
    0, 1, 1, 1, 0, 0, 1, 0, 0, 1, 1, 1, 0, 0, 0, 1, 0, 1, 1, 1, 0, 0, 0, 0, 0, 1, 1, 0, 1, 1, 0, 1, 0, 1, 1, 0, 1, 1, 0, 0, 0, 1, 1, 0, 1, 0, 1, 0, 0, 1,
    1, 0, 1, 0, 0, 1, 0, 1, 1, 0, 1, 0, 0, 0, 0, 1, 1, 0, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0,
};

static uint8_t node_pool[20 * (695 + 366)] = {0};
static bool enable_simulated_signals = false; // 是否启用模拟信号
static volatile uint32_t simulated_base_signal_index = 0, 
                        simulated_neck_signal_index = 0; // 模拟信号索引

// 信号稳定性检查结构
typedef struct {
    int last_stable_signal;      // 上次稳定的信号值
    uint32_t last_change_time;   // 上次信号变化时间
    uint32_t debounce_time_ms;   // 去抖时间（毫秒）
    bool is_initialized;         // 是否已初始化
} encoder_signal_filter_t;

static encoder_signal_filter_t signal_filters[2] = {
    {0, 0, 1, false},  // MOTOR_NECK: 1ms去抖时间
    {0, 0, 1, false}   // MOTOR_BASE: 1ms去抖时间
};

void encoder_enable_simulated_signals(bool enable)
{
    enable_simulated_signals = enable;
}

uint8_t read_simulated_signal(uint8_t motor_index, int direction)
{
    if(motor_index == MOTOR_NECK)
    {
        if(direction == Motor_Direction_Forward && simulated_neck_signal_index < (180 - 1) * NECK_MOTOR_STEPS_PER_DEGREE / ENCODER_UNITS_PER_DEG)
        {
            simulated_neck_signal_index++;
        }
        else if(direction == Motor_Direction_Backward && simulated_neck_signal_index > 0)
        {
            simulated_neck_signal_index--;
        }

        return neck_encoder_map_data[(int)(simulated_neck_signal_index / NECK_MOTOR_STEPS_PER_DEGREE / ENCODER_UNITS_PER_DEG)];
    }
    else if(motor_index == MOTOR_BASE)
    {
        if(direction == Motor_Direction_Forward && simulated_base_signal_index < (350 - 1) * BASE_MOTOR_STEPS_PER_DEGREE)
        {
            simulated_base_signal_index++;
        }
        else if(direction == Motor_Direction_Backward && simulated_base_signal_index > 0)
        {
            simulated_base_signal_index--;
        }

        return base_encoder_map_data[(int)(simulated_base_signal_index / BASE_MOTOR_STEPS_PER_DEGREE / ENCODER_UNITS_PER_DEG)];
    }
    
    return 0; // Invalid motor index
}

encoder_map_handle_t base_encoder_map_handle, neck_encoder_map_handle;
encoder_handle_t encoder_neck_handle, encoder_base_handle;
encoder_position_t encoder_neck_position, encoder_base_position;
encoder_result_t encoder_neck_search_result, encoder_base_search_result;

encoder_handle_t encoder_get_handle(uint8_t index)
{
    if(index == MOTOR_NECK)
    {
        return encoder_neck_handle;
    }
    else if(index == MOTOR_BASE)
    {
        return encoder_base_handle;
    }

    return NULL; // Invalid motor index
}

void encoder_init(void)
{
    encoder_config_t encoder_config;
    encoder_config.tracking_lost_threshold = 3; // 连续1次失败切换搜索模式
    encoder_config.backlash_compensation = 0;   // 默认回差补偿
    
    base_encoder_map_handle = abs_encoder_create_map_with_pool(base_encoder_map_data, 350, 11, node_pool, 695);
    neck_encoder_map_handle = abs_encoder_create_map_with_pool(neck_encoder_map_data, 180, 9, node_pool + (695 * 20), 366);

    // 创建喷嘴编码器并配置运行时校准
    encoder_config.motor_steps_per_unit = NECK_MOTOR_STEPS_PER_DEGREE / ENCODER_UNITS_PER_DEG;          // 20.16步
    encoder_neck_handle = abs_encoder_create(neck_encoder_map_handle, &encoder_config);
    abs_encoder_config_sampling_point(encoder_neck_handle, 0.3f, 0, 22);               // 设置喷嘴编码器采样点
    
    // 创建底座编码器并配置运行时校准
    encoder_config.motor_steps_per_unit = BASE_MOTOR_STEPS_PER_DEGREE / ENCODER_UNITS_PER_DEG;            // 28.8步
    encoder_base_handle = abs_encoder_create(base_encoder_map_handle, &encoder_config);
    abs_encoder_config_sampling_point(encoder_base_handle, 0.3f, 0, 29);                 // 设置底座编码器采样点
    
    // 配置运行时校准参数
    runtime_calibration_config_t runtime_config;
    runtime_config.tolerance_ratio = 0.3f;                  // 校准容差比例 (0.0-1.0)
    runtime_config.min_samples_for_calibration = 10;        // 校准所需最小样本数
    
    runtime_config.max_samples_for_analysis = 60;           // 分析用最大样本数
    runtime_config.nominal_steps_per_unit = NECK_MOTOR_STEPS_PER_DEGREE / ENCODER_UNITS_PER_DEG;
    abs_encoder_config_runtime_calibration(encoder_neck_handle, &runtime_config);

    runtime_config.max_samples_for_analysis = 120;           // 分析用最大样本数
    runtime_config.nominal_steps_per_unit = BASE_MOTOR_STEPS_PER_DEGREE / ENCODER_UNITS_PER_DEG;
    abs_encoder_config_runtime_calibration(encoder_base_handle, &runtime_config);
}

int encoder_get_position(uint8_t index)
{
    if(index == MOTOR_NECK)
    {
        return encoder_neck_position.absolute_position;
    }
    else if(index == MOTOR_BASE)
    {
        return encoder_base_position.absolute_position;
    }

    return 0; // Invalid motor index
}

encoder_state_t encoder_get_state(uint8_t index)
{
    encoder_position_t position;
    encoder_handle_t encoder_handle = encoder_get_handle(index);
    
    if(encoder_handle)
    {
        if(abs_encoder_get_current_position(encoder_handle, &position))
        {
            return position.state;
        }
    }

    return ENCODER_STATE_ERROR; // Invalid motor index or failed to get position
}

encoder_result_t encoder_get_search_result(uint8_t index)
{
    if(index == MOTOR_NECK)
    {
        return encoder_neck_search_result;
    }
    else if(index == MOTOR_BASE)
    {
        return encoder_base_search_result;
    }

    return ENCODER_RESULT_ERROR_INVALID_SIGNAL; // Invalid motor index
}

const char *encoder_get_state_string(encoder_state_t state)
{
    switch(state)
    {
    case ENCODER_STATE_UNINITIALIZED: return "Uninitialized";
    case ENCODER_STATE_SEARCHING:     return "Searching";
    case ENCODER_STATE_TRACKING:      return "Tracking";
    case ENCODER_STATE_ERROR:         return "Error";
    }

    return "Unknown";
}

const char *encoder_get_search_result_string(encoder_result_t result)
{
    switch(result)
    {
    case ENCODER_RESULT_OK:                   return "OK";
    case ENCODER_RESULT_SEARCHING:            return "Searching";
    case ENCODER_RESULT_POSITION_FOUND:       return "Position Found";
    case ENCODER_RESULT_TRACKING_UPDATED:     return "Tracking Updated";
    case ENCODER_RESULT_ERROR_INVALID_SIGNAL: return "Invalid Signal";
    case ENCODER_RESULT_ERROR_LOST_TRACKING:  return "Lost Tracking";
    case ENCODER_RESULT_ERROR_TIMEOUT:        return "Timeout";
    }

    return "Unknown Result";
}

// 获取校准状态字符串（用于调试）
const char* encoder_get_calibration_state_string(runtime_calibration_state_t state)
{
    switch(state)
    {
        case RUNTIME_CALIBRATION_UNCALIBRATED: return "UNCALIBRATED";
        case RUNTIME_CALIBRATION_COLLECTING:   return "COLLECTING";
        case RUNTIME_CALIBRATION_CALIBRATED:   return "CALIBRATED";
        case RUNTIME_CALIBRATION_FAILED:       return "FAILED";
        default:                               return "UNKNOWN";
    }
}

// 在主循环中调用的处理函数
void encoder_process_samples(void)
{
    encoder_sampled_data_t sampled_data;
    uint32_t start_time, end_time;
    uint8_t value;

    // 处理所有待处理的采样
    while(stepper_motor_get_encoder_data(&sampled_data, pdMS_TO_TICKS(1)))
    {
        // 从队列取出数据
        encoder_handle_t encoder_handle = encoder_get_handle(sampled_data.motor_index);
        if(enable_simulated_signals)
        {
            sampled_data.sampled_signal = read_simulated_signal(sampled_data.motor_index, sampled_data.direction);
        }

        // 更新读指针
        if(sampled_data.motor_index == MOTOR_NECK && encoder_neck_handle)
        {
            encoder_neck_search_result = abs_encoder_process_step_with_signal(
                encoder_neck_handle,
                (sampled_data.direction == (int)Motor_Direction_Forward) ? 1 : -1,
                sampled_data.sampled_signal, &encoder_neck_position);
        }
        else if(sampled_data.motor_index == MOTOR_BASE && encoder_base_handle)
        {
            encoder_base_search_result = abs_encoder_process_step_with_signal(
                encoder_base_handle,
                (sampled_data.direction == (int)Motor_Direction_Forward) ? 1 : -1,
                sampled_data.sampled_signal, &encoder_base_position);
        }
    }
}

// 获取运行时校准状态
void encoder_get_runtime_calibration_status(uint8_t motor_index, runtime_calibration_result_t *result)
{
    if(!result) return;
    
    if(motor_index == MOTOR_NECK && encoder_neck_handle)
    {
        abs_encoder_get_runtime_calibration_result(encoder_neck_handle, result);
    }
    else if(motor_index == MOTOR_BASE && encoder_base_handle)
    {
        abs_encoder_get_runtime_calibration_result(encoder_base_handle, result);
    }
}
