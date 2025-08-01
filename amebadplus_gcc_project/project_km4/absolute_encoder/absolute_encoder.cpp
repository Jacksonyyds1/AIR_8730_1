#include "absolute_encoder.h"
#include "encoder_map.hpp"
#include "smart_search_manager.hpp"
#include "position_tracker.hpp"
#include "input_preprocessor.hpp"
#include "runtime_calibrator.hpp"
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <cstdio>
#include <new>
#include <vector>

using namespace AbsoluteEncoder;

int absolute_encoder_log_level = 0;

/**
 * 编码器图案句柄实现
 */
struct encoder_map_handle_s
{
    EncoderMap *encoder_map;
};

/**
 * 编码器句柄实现
 */
struct encoder_handle_s
{
    // 共享的编码器图案
    encoder_map_handle_t map_handle;

    // 各功能模块
    InputPreprocessor *input_processor;
    SmartSearchManager *search_manager;
    PositionTracker *position_tracker;
    RuntimeCalibrator *runtime_calibrator;

    // 配置和状态
    encoder_config_t config;
    encoder_state_t current_state;
    encoder_position_t current_position;
    encoder_stats_t stats;

    // 运行时状态
    uint32_t last_signal_timestamp;
    bool debug_enabled;
};

/**
 * 内部辅助函数声明
 */
static encoder_result_t handle_search_mode(encoder_handle_t handle, uint8_t signal_bit, SearchDirection search_direction);
static encoder_result_t handle_tracking_mode(encoder_handle_t handle, uint8_t signal_bit, SearchDirection track_direction);
static void transition_to_tracking_mode(encoder_handle_t handle, uint32_t found_position, SearchDirection search_direction);
static void transition_to_search_mode(encoder_handle_t handle);

// === 日志等级设置 ===

void abs_encoder_set_log_level(int level)
{
    absolute_encoder_log_level = level;
}

// === 编码器图案对象管理 ===

encoder_map_handle_t abs_encoder_create_map(const uint8_t *pattern_data, uint32_t pattern_length, uint8_t max_search_depth)
{
    if(!pattern_data || pattern_length == 0 || max_search_depth == 0)
    {
        return NULL;
    }

    encoder_map_handle_t map_handle = (encoder_map_handle_t)malloc(sizeof(struct encoder_map_handle_s));
    if(!map_handle)
    {
        return NULL;
    }

    // 将C数组转换为std::vector
    std::vector<uint8_t> pattern(pattern_data, pattern_data + pattern_length);
    map_handle->encoder_map = new(std::nothrow) EncoderMap(pattern_data, pattern_length, max_search_depth);
    if(!map_handle->encoder_map)
    {
        free(map_handle);
        return NULL;
    }

    return map_handle;
}

encoder_map_handle_t abs_encoder_create_map_with_pool(const uint8_t *pattern_data, uint32_t pattern_length, uint8_t max_search_depth, void *node_pool, uint16_t pool_size)
{
    if(!pattern_data || pattern_length == 0 || max_search_depth == 0 || !node_pool || pool_size == 0)
    {
        return NULL;
    }

    encoder_map_handle_t map_handle = (encoder_map_handle_t)malloc(sizeof(struct encoder_map_handle_s));
    if(!map_handle)
    {
        return NULL;
    }

    map_handle->encoder_map = new(std::nothrow) EncoderMap(pattern_data, pattern_length, max_search_depth, (TreeNode *)node_pool, pool_size);
    if(!map_handle->encoder_map)
    {
        free(map_handle);
        return NULL;
    }

    return map_handle;
}

void abs_encoder_destroy_map(encoder_map_handle_t map_handle)
{
    if(map_handle)
    {
        delete map_handle->encoder_map;
        free(map_handle);
    }
}

// === 编码器对象管理 ===

encoder_handle_t abs_encoder_create(encoder_map_handle_t map_handle, const encoder_config_t *config)
{
    if(!map_handle || !config)
    {
        return NULL;
    }

    encoder_handle_t handle = (encoder_handle_t)malloc(sizeof(struct encoder_handle_s));
    if(!handle)
    {
        return NULL;
    }

    // 初始化基本信息
    memset(handle, 0, sizeof(struct encoder_handle_s));
    handle->map_handle = map_handle;
    handle->config = *config;
    handle->current_state = ENCODER_STATE_SEARCHING;
    handle->current_position.state = ENCODER_STATE_SEARCHING;

    // 创建输入预处理器
    InputPreprocessor::EdgeSamplingConfig edge_config;
    edge_config.steps_per_encoder_unit = config->motor_steps_per_unit;
    edge_config.backlash_compensation = config->backlash_compensation;  // 默认回差补偿步数
    edge_config.edge_prediction_tolerance = 0.15f;
    edge_config.enable_adaptive_prediction = true;
    edge_config.min_steps_for_edge = 5;
    edge_config.max_steps_for_edge = static_cast<uint16_t>(config->motor_steps_per_unit * 2);

    handle->input_processor = new(std::nothrow) InputPreprocessor(edge_config);
    if(!handle->input_processor)
    {
        free(handle);
        return NULL;
    }

    // 创建搜索管理器
    handle->search_manager = new(std::nothrow) SmartSearchManager(*map_handle->encoder_map);
    if(!handle->search_manager)
    {
        delete handle->input_processor;
        free(handle);
        return NULL;
    }

    // 创建位置跟踪器
    const uint8_t *pattern_data = map_handle->encoder_map->get_pattern_data();
    uint16_t pattern_length = map_handle->encoder_map->get_pattern_length();
    handle->position_tracker = new(std::nothrow) PositionTracker(*map_handle->encoder_map, pattern_data, pattern_length, config->enable_ring_mode);
    if(!handle->position_tracker)
    {
        delete handle->input_processor;
        delete handle->search_manager;
        free(handle);
        return NULL;
    }

    // 创建运行时校准器（默认禁用，需要手动启用）
    handle->runtime_calibrator = nullptr;

    return handle;
}

void abs_encoder_destroy(encoder_handle_t handle)
{
    if(handle)
    {
        delete handle->input_processor;
        delete handle->search_manager;
        delete handle->position_tracker;
        delete handle->runtime_calibrator;  // 安全删除
        free(handle);
    }
}

// === 核心信号处理 ===

encoder_result_t abs_encoder_process_step_with_signal(encoder_handle_t handle, int8_t direction,
        uint8_t sensor_signal, encoder_position_t *position_info)
{
    if(!handle || !position_info)
    {
        return ENCODER_RESULT_ERROR_INVALID_SIGNAL;
    }
    
    encoder_result_t process_result = ENCODER_RESULT_OK;
    InputPreprocessor::StepDirection step_dir = (direction > 0) ?
            InputPreprocessor::StepDirection::FORWARD :
            InputPreprocessor::StepDirection::BACKWARD;

    // 调用输入预处理器进行基于采样点的信号处理
    auto result = handle->input_processor->process_step_with_signal(step_dir, sensor_signal);

    // 向校准器提供原始步进数据（独立于采样点判定）
    if(handle->runtime_calibrator)
    {
        RuntimeCalibrator::RawStepData raw_data;
        raw_data.sensor_signal = sensor_signal;
        
        handle->runtime_calibrator->add_raw_step_data(raw_data);
    }
    
    // 检查是否到达采样点
    if(result.should_sample_signal)
    {
        // 更新统计信息
        handle->stats.total_signals_processed++;

        // 如果有编码器单位变化，更新位置信息
        if(result.encoder_unit_change != 0 && handle->current_state == ENCODER_STATE_TRACKING)
        {
            handle->current_position.relative_position_change += result.encoder_unit_change;

            LOGI("Encoder unit completed: change=%d, total_change=%d, signal=%d",
                 result.encoder_unit_change, handle->current_position.relative_position_change, result.current_signal_value);
        }

        // 将输入预处理器的方向转换为搜索方向
        SearchDirection search_direction = (direction > 0) ? SearchDirection::FORWARD : SearchDirection::BACKWARD;

        // 根据当前状态处理完成的编码器单位
        switch(handle->current_state)
        {
        case ENCODER_STATE_SEARCHING:
            process_result = handle_search_mode(handle, result.current_signal_value, search_direction);
            break;

        case ENCODER_STATE_TRACKING:
            process_result = handle_tracking_mode(handle, result.current_signal_value, search_direction);
            break;

        case ENCODER_STATE_ERROR:
            // 错误状态下尝试重新搜索
            transition_to_search_mode(handle);
            process_result = ENCODER_RESULT_SEARCHING;
            break;

        default:
            handle->current_state = ENCODER_STATE_ERROR;
            process_result = ENCODER_RESULT_ERROR_INVALID_SIGNAL;
            break;
        }
    }
    
    // 跟踪模式下估算当前位置
    if(handle->current_state == ENCODER_STATE_TRACKING)
    {
        handle->current_position.estimated_position = handle->current_position.absolute_position + handle->input_processor->get_estimated_unit_offset();
    }
    
    // 输出当前位置信息
    *position_info = handle->current_position;
    return process_result;
}

// === 状态查询接口 ===

bool abs_encoder_get_current_position(encoder_handle_t handle, encoder_position_t *position_info)
{
    if(!handle || !position_info)
    {
        return false;
    }

    *position_info = handle->current_position;
    return true;
}

bool abs_encoder_reset(encoder_handle_t handle)
{
    if(!handle)
    {
        return false;
    }

    // 重置所有模块
    handle->input_processor->reset();
    handle->search_manager->reset();
    handle->position_tracker->reset();

    // 重置状态
    handle->current_state = ENCODER_STATE_SEARCHING;
    memset(&handle->current_position, 0, sizeof(encoder_position_t));
    handle->current_position.state = ENCODER_STATE_SEARCHING;
    memset(&handle->stats, 0, sizeof(encoder_stats_t));

    return true;
}

bool abs_encoder_force_search_mode(encoder_handle_t handle)
{
    if(!handle)
    {
        return false;
    }

    transition_to_search_mode(handle);
    return true;
}

// === 内部状态处理函数 ===

static encoder_result_t handle_search_mode(encoder_handle_t handle, uint8_t signal_bit, SearchDirection search_direction)
{
    handle->stats.search_attempts++;

    // 使用智能搜索管理器进行搜索，传递实际的搜索方向
    uint16_t found_position;
    SearchResult search_result = handle->search_manager->add_bit_and_search(
            signal_bit, search_direction, found_position);

    switch(search_result)
    {
    case SearchResult::FOUND:
        // 找到位置，切换到跟踪模式
        transition_to_tracking_mode(handle, found_position, search_direction);
        handle->current_position.absolute_position = found_position;
        return ENCODER_RESULT_POSITION_FOUND;

    case SearchResult::PARTIAL:
        // 部分匹配，继续搜索
        return ENCODER_RESULT_SEARCHING;

    case SearchResult::NOT_FOUND:
        // 未找到匹配，继续搜索
        return ENCODER_RESULT_SEARCHING;
    }

    return ENCODER_RESULT_SEARCHING;
}

static encoder_result_t handle_tracking_mode(encoder_handle_t handle, uint8_t signal_bit, SearchDirection track_direction)
{
    handle->stats.tracking_updates++;

    // 使用位置跟踪器进行跟踪，传递实际的运动方向
    uint16_t tracked_position;
    ValidationResult validation_result = handle->position_tracker->track_position(
            signal_bit, track_direction, tracked_position);

    switch(validation_result)
    {
    case ValidationResult::VALID:
        // 跟踪成功
        handle->current_position.absolute_position = tracked_position;
        // 根据方向设置相对位置变化
        handle->current_position.relative_position_change = (track_direction == SearchDirection::FORWARD) ? 1 : -1;
        return ENCODER_RESULT_TRACKING_UPDATED;

    case ValidationResult::INVALID:
        // 跟踪失败，切换到搜索模式
        transition_to_search_mode(handle);
        handle->stats.error_count++;
        return ENCODER_RESULT_ERROR_LOST_TRACKING;
    }

    return ENCODER_RESULT_OK;
}

static void transition_to_tracking_mode(encoder_handle_t handle, uint32_t found_position, SearchDirection search_direction)
{
    handle->current_state = ENCODER_STATE_TRACKING;
    handle->current_position.state = ENCODER_STATE_TRACKING;
    handle->current_position.absolute_position = found_position;
    handle->current_position.relative_position_change = 0;

    // 初始化位置跟踪器，使用实际的搜索方向
    handle->position_tracker->initialize_position(found_position, search_direction);

    LOGI("Encoder: Switched to tracking mode at position %u\n", found_position);
}

static void transition_to_search_mode(encoder_handle_t handle)
{
    handle->current_state = ENCODER_STATE_SEARCHING;
    handle->current_position.state = ENCODER_STATE_SEARCHING;
    // handle->current_position.absolute_position = 0;  // 不重置绝对位置，保持上次搜索/跟踪结果
    handle->current_position.relative_position_change = 0;

    // 重置搜索管理器
    handle->search_manager->reset();

    LOGI("Encoder: Switched to search mode\n");
}

// === 运行时校准接口实现 ===

bool abs_encoder_config_runtime_calibration(encoder_handle_t handle, const runtime_calibration_config_t *config)
{
    if(!handle || !config)
    {
        return false;
    }

    // 如果校准器不存在，创建它
    if(!handle->runtime_calibrator)
    {
        RuntimeCalibrator::CalibrationConfig calibration_config;
        calibration_config.nominal_steps_per_unit = config->nominal_steps_per_unit;
        calibration_config.tolerance_ratio = config->tolerance_ratio;
        calibration_config.min_samples_for_calibration = config->min_samples_for_calibration;
        calibration_config.max_samples_for_analysis = config->max_samples_for_analysis;

        handle->runtime_calibrator = new(std::nothrow) RuntimeCalibrator(calibration_config);
        if(!handle->runtime_calibrator)
        {
            return false;
        }
    }
    else
    {
        // 更新现有校准器的配置
        RuntimeCalibrator::CalibrationConfig calibration_config;
        calibration_config.nominal_steps_per_unit = config->nominal_steps_per_unit;
        calibration_config.tolerance_ratio = config->tolerance_ratio;
        calibration_config.min_samples_for_calibration = config->min_samples_for_calibration;
        calibration_config.max_samples_for_analysis = config->max_samples_for_analysis;

        handle->runtime_calibrator->update_config(calibration_config);
    }

    return true;
}

bool abs_encoder_get_runtime_calibration_result(encoder_handle_t handle, runtime_calibration_result_t *result)
{
    if(!handle || !result)
    {
        return false;
    }

    if(handle->runtime_calibrator)
    {
        auto calibration_result = handle->runtime_calibrator->get_calibration_result();

        // 转换内部结果到外部接口
        result->state = static_cast<runtime_calibration_state_t>(calibration_result.state);
        result->calibrated_steps_per_unit = calibration_result.calibrated_steps_per_unit;
        result->confidence_level = calibration_result.confidence_level;
        result->sample_count = calibration_result.sample_count;
        result->step_variance = calibration_result.step_variance;
    }
    else
    {
        // 无校准器时返回默认值
        result->state = RUNTIME_CALIBRATION_UNCALIBRATED;
        result->calibrated_steps_per_unit = handle->config.motor_steps_per_unit;
        result->confidence_level = 0.0f;
        result->sample_count = 0;
        result->step_variance = 0.0f;
    }

    return true;
}

bool abs_encoder_reset_runtime_calibration(encoder_handle_t handle)
{
    if(!handle)
    {
        return false;
    }

    if(handle->runtime_calibrator)
    {
        handle->runtime_calibrator->reset();
        LOGD("Runtime calibration reset");
        return true;
    }

    return false;
}

bool abs_encoder_force_runtime_calibration_analysis(encoder_handle_t handle)
{
    if(!handle)
    {
        return false;
    }

    if(handle->runtime_calibrator)
    {
        bool success = handle->runtime_calibrator->force_analyze();
        LOGI("Runtime calibration analysis forced: %s", success ? "success" : "failed");
        return success;
    }

    return false;
}

runtime_calibration_state_t abs_encoder_get_runtime_calibration_state(encoder_handle_t handle)
{
    if(!handle)
    {
        return RUNTIME_CALIBRATION_UNCALIBRATED;
    }

    if(handle->runtime_calibrator)
    {
        auto state = handle->runtime_calibrator->get_state();
        return static_cast<runtime_calibration_state_t>(state);
    }

    return RUNTIME_CALIBRATION_UNCALIBRATED;
}

bool abs_encoder_get_sampling_config(encoder_handle_t handle, encoder_sampling_config_t *config)
{
    if(!handle || !config)
    {
        return false;
    }

    // 从输入预处理器获取当前配置（简化实现）
    config->sampling_point_ratio = 0.5f; // 默认值，实际应该从input_processor获取
    config->max_steps_for_edge = static_cast<uint32_t>(handle->config.motor_steps_per_unit * 2);

    return true;
}

bool abs_encoder_needs_recalibration(encoder_handle_t handle)
{
    if(!handle)
    {
        return false;
    }

    if(handle->runtime_calibrator)
    {
        return handle->runtime_calibrator->needs_recalibration();
    }

    return false;
}

// === 采样点配置接口 ===

bool abs_encoder_config_sampling_point(encoder_handle_t handle, float sampling_ratio, uint16_t symertric_compensation, uint32_t max_steps)
{
    if(!handle)
    {
        return false;
    }

    // 参数验证
    if(sampling_ratio < 0.0f || sampling_ratio > 1.0f)
    {
        LOGE("Invalid sampling ratio: %f (must be 0.0-1.0)", sampling_ratio);
        return false;
    }

    // 更新输入预处理器的配置
    AbsoluteEncoder::InputPreprocessor::EdgeSamplingConfig config;
    config.steps_per_encoder_unit = handle->config.motor_steps_per_unit;
    config.sampling_point_ratio = sampling_ratio;
    config.max_steps_for_edge = max_steps;
    config.backlash_compensation = 0;  // 根据需要设置
    config.symertric_compensation = symertric_compensation;

    handle->input_processor->update_config(config);

    LOGD("Sampling point configured: ratio=%.3f, max_steps=%u",
         sampling_ratio, max_steps);

    return true;
}

bool abs_encoder_add_calibration_step_data(encoder_handle_t handle, uint8_t sensor_signal)
{
    if(!handle)
    {
        return false;
    }

    if(handle->runtime_calibrator)
    {
        RuntimeCalibrator::RawStepData raw_data;
        raw_data.sensor_signal = sensor_signal;
        
        handle->runtime_calibrator->add_raw_step_data(raw_data);
        return true;
    }

    return false;
}
