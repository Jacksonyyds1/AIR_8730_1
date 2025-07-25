#include "absolute_encoder.h"
#include "input_preprocessor.hpp"
#include <algorithm>
#include <cstring>
#include <cmath>

namespace AbsoluteEncoder
{

InputPreprocessor::InputPreprocessor(const EdgeSamplingConfig &config)
    : config_(config),
        current_state_(ProcessingState::INITIALIZING),
        current_direction_(StepDirection::FORWARD),
        last_direction_(StepDirection::FORWARD),
        current_step_position_(0),
        steps_since_last_edge_(0),
        backlash_compensation_steps_(0),
        already_sampled_at_unit_(false),
        last_sensor_signal_(0),
        total_edges_detected_(0),
        successful_predictions_(0),
        direction_changes_(0)
{
    LOGD("EdgeSampling InputPreprocessor initialized: steps_per_unit=%.2f, backlash=%d",
            config_.steps_per_encoder_unit, config_.backlash_compensation);
}

InputPreprocessor::ProcessingResult InputPreprocessor::process_step_with_signal(StepDirection direction, uint8_t sensor_signal)
{
    UNUSED(sensor_signal);
    ProcessingResult result;

    // 更新当前步数位置
    current_step_position_ += (direction == StepDirection::FORWARD) ? 1 : -1;
    steps_since_last_edge_++;

    // 检查方向变化
    bool direction_changed = (direction != current_direction_);
    if(direction_changed)
    {
        if(handle_direction_change(direction))
        {
            // 需要回差补偿
            result.new_state = ProcessingState::BACKLASH_COMPENSATING;
            current_state_ = result.new_state;
            return result;
        }
        else
        {
            // 方向变化但不需要回差补偿，直接转入正常跟踪状态
            result.new_state = (current_direction_ == StepDirection::FORWARD) ?
                                ProcessingState::TRACKING_FORWARD : ProcessingState::TRACKING_BACKWARD;
            current_state_ = result.new_state;
        }
    }

    // 根据当前状态处理
    switch(current_state_)
    {
    case ProcessingState::INITIALIZING:
        result = handle_initialization(sensor_signal);
        break;

    case ProcessingState::BACKLASH_COMPENSATING:
        result = handle_backlash_compensation(sensor_signal);
        break;

    case ProcessingState::TRACKING_FORWARD:
    case ProcessingState::TRACKING_BACKWARD:
        result = handle_tracking(sensor_signal);
        break;
    }

    // 更新状态
    current_state_ = result.new_state;
    current_direction_ = direction;
    last_sensor_signal_ = sensor_signal;

    return result;
}

InputPreprocessor::ProcessingResult InputPreprocessor::handle_initialization(uint8_t sensor_signal)
{
    ProcessingResult result;

    // 优先检测边沿，如果有边沿则直接以边沿为基准
    bool edge_detected = detect_signal_edge(sensor_signal);
    bool threshold_reached = steps_since_last_edge_ >= static_cast<int32_t>(config_.steps_per_encoder_unit * 0.1f);

    if(edge_detected || threshold_reached)
    {
        // 建立初始采样基准点
        update_sampling_reference(current_step_position_, sensor_signal);

        result.unit_completed = true;
        result.current_signal_value = sensor_signal;
        result.encoder_unit_change = 1;  // 第一个单位
        result.new_state = (current_direction_ == StepDirection::FORWARD) ?
                            ProcessingState::TRACKING_FORWARD : ProcessingState::TRACKING_BACKWARD;

        total_edges_detected_++;
        steps_since_last_edge_ = 0;
        already_sampled_at_unit_ = false;

        // 边沿检测的精度更高
        result.prediction_accuracy = edge_detected ? 1.0f : 0.7f;
    }
    else
    {
        result.new_state = ProcessingState::INITIALIZING;
    }

    return result;
}

InputPreprocessor::ProcessingResult InputPreprocessor::handle_tracking(uint8_t sensor_signal)
{
    ProcessingResult result;
    result.unit_completed = false;
    result.encoder_unit_change = 0;
    result.should_sample_signal = false;
    result.new_state = current_state_;
    result.prediction_accuracy = get_average_prediction_accuracy();

    int32_t direction_multiplier = (current_direction_ == StepDirection::FORWARD) ? 1 : -1;

    // 计算在当前编码器单位区间内的位置
    int32_t steps_from_last_unit = steps_since_last_edge_;
    float unit_size = config_.steps_per_encoder_unit;
    float sampling_offset = unit_size * config_.sampling_point_ratio;
    int32_t target_sampling_position = static_cast<int32_t>(sampling_offset);

    // 执行对称补偿（如果配置了）
    if(config_.symertric_compensation > 0)
    {
        if(sensor_signal)
        {
            steps_from_last_unit += config_.symertric_compensation;
        }
        else
        {
            steps_from_last_unit -= config_.symertric_compensation;
        }
    }

    // 检查是否到达采样点（在一个完整编码器单位周期内）
    bool at_sampling_point = (std::abs(steps_from_last_unit - target_sampling_position) <= 1);
    bool unit_complete = (steps_from_last_unit >= static_cast<int32_t>(unit_size));
    bool timeout = (steps_from_last_unit >= config_.max_steps_for_edge);
    bool edge_detected = detect_signal_edge(sensor_signal);

    if(edge_detected || unit_complete || timeout)
    {
        // 检测到边沿完成一个编码器单位或超时
        result.should_sample_signal = false;
        result.current_signal_value = sensor_signal;
        result.unit_completed = true;
        result.encoder_unit_change = direction_multiplier;

        // 更新采样基准点
        update_sampling_reference(current_step_position_, sensor_signal);

        // 更新统计信息
        total_edges_detected_++;
        steps_since_last_edge_ = 0;       // 重置区间计数，开始新的编码器单位
        already_sampled_at_unit_ = false; // 重置采样标记

        if(unit_complete)
        {
            successful_predictions_++;
            result.prediction_accuracy = 1.0f;  // 正常完成
        }
        else
        {
            result.prediction_accuracy = 0.5f;  // 超时完成
        }
    }
    else if(at_sampling_point && !already_sampled_at_unit_)
    {
        // 到达采样点但还没完成整个编码器单位，进行信号采样但不完成单位
        already_sampled_at_unit_ = true;  // 标记已在当前单位采样
        result.should_sample_signal = true;
        result.current_signal_value = sensor_signal;
        // 不设置 unit_completed = true，等到完成整个编码器单位
    }

    return result;
}

InputPreprocessor::ProcessingResult InputPreprocessor::handle_backlash_compensation(uint8_t sensor_signal)
{
    UNUSED(sensor_signal);
    ProcessingResult result;

    if(backlash_compensation_steps_ > 0)
    {
        backlash_compensation_steps_--;
        result.new_state = ProcessingState::BACKLASH_COMPENSATING;

        LOGV("Backlash compensation: %d steps remaining",
                (int)backlash_compensation_steps_);
    }
    else
    {
        // 回差补偿完成，继续正常跟踪
        result.new_state = (current_direction_ == StepDirection::FORWARD) ?
                            ProcessingState::TRACKING_FORWARD : ProcessingState::TRACKING_BACKWARD;

        LOGD("Backlash compensation completed, starting edge sync");
    }

    return result;
}

bool InputPreprocessor::detect_signal_edge(uint8_t current_signal)
{
    // 检测 0->1 或 1->0 的跳变
    return (current_signal != last_sensor_signal_);
}

int32_t InputPreprocessor::calculate_based_edge_for_sample(StepDirection from_direction, StepDirection to_direction)
{
    // 预测换向后的第一个边沿位置
    // 这里使用旧方向上的下一个预测位置作为新方向的第一个边沿

    if(from_direction == to_direction)
    {
        // 方向未变，使用正常预测
        int32_t direction_multiplier = (to_direction == StepDirection::FORWARD) ? 1 : -1;
        return current_reference_.step_position +
                direction_multiplier * static_cast<int32_t>(config_.steps_per_encoder_unit);
    }
    else
    {
        // 方向改变，使用当前进度计算预测位置
        float progress_ratio = (float)steps_since_last_edge_ / config_.steps_per_encoder_unit;
        float remaining_ratio = 1.0f - progress_ratio;

        // 新方向的第一个边沿应该在剩余距离处
        int32_t direction_multiplier = (to_direction == StepDirection::FORWARD) ? 1 : -1;
        return current_step_position_ +
                direction_multiplier * static_cast<int32_t>(remaining_ratio * config_.steps_per_encoder_unit);
    }
}

bool InputPreprocessor::handle_direction_change(StepDirection new_direction)
{
    if(new_direction == current_direction_)
    {
        return false;  // 方向未变
    }

    direction_changes_++;
    last_direction_ = current_direction_;

    // 预测新方向使用的基准边沿位置
    steps_since_last_edge_ = calculate_based_edge_for_sample(current_direction_, new_direction);

    // 无论如何，方向变化后都标记当前单位已采样
    already_sampled_at_unit_ = true;

    // 检查是否需要回差补偿
    if(config_.backlash_compensation > 0)
    {
        backlash_compensation_steps_ = config_.backlash_compensation;
        return true;  // 需要回差补偿
    }
    else
    {
        return false;  // 不需要回差补偿
    }
}

void InputPreprocessor::update_sampling_reference(int32_t step_pos, uint8_t signal_val)
{
    current_reference_.step_position = step_pos;
    current_reference_.signal_value = signal_val;
    current_reference_.encoder_unit_position = calculate_encoder_unit_change();
}

int32_t InputPreprocessor::calculate_encoder_unit_change()
{
    // 基于当前基准点计算编码器单位位置
    return current_reference_.encoder_unit_position +
            ((current_direction_ == StepDirection::FORWARD) ? 1 : -1);
}

void InputPreprocessor::update_prediction_statistics(int32_t predicted_pos, int32_t actual_pos)
{
    float accuracy = 1.0f - (float)std::abs(actual_pos - predicted_pos) / config_.steps_per_encoder_unit;
    accuracy = std::max(0.0f, std::min(1.0f, accuracy));  // 限制在 [0,1] 范围

    prediction_accuracy_history_.push_back(accuracy);

    // 保持历史记录在合理大小
    if(prediction_accuracy_history_.size() > 20)
    {
        prediction_accuracy_history_.pop_front();
    }
}

bool InputPreprocessor::should_trigger_edge_sampling()
{
    // 计算在当前编码器单位区间内的采样点位置
    // 采样永远都在采样点上触发，不在单位边界触发
    int32_t steps_from_last_edge = steps_since_last_edge_;

    // 计算采样点在编码器单位中的位置
    float sampling_offset = config_.steps_per_encoder_unit * config_.sampling_point_ratio;
    int32_t target_sampling_position = static_cast<int32_t>(sampling_offset);

    // 只在采样点位置触发（允许±1步的容差）
    return (std::abs(steps_from_last_edge - target_sampling_position) <= 1);
}

void InputPreprocessor::adaptive_adjust_prediction()
{
    if(prediction_accuracy_history_.size() < 5)
    {
        return;  // 历史数据不足
    }

    // 计算平均预测精度
    float total_accuracy = 0.0f;
    for(float accuracy : prediction_accuracy_history_)
    {
        total_accuracy += accuracy;
    }
    float avg_accuracy = total_accuracy / prediction_accuracy_history_.size();

    // 如果精度较低，可以考虑调整预测参数
    if(avg_accuracy < 0.8f)
    {
        LOGW("Low prediction accuracy: %.3f, may need parameter adjustment",
                avg_accuracy);
    }
}

float InputPreprocessor::get_average_prediction_accuracy() const
{
    if(prediction_accuracy_history_.empty())
    {
        return 0.0f;
    }

    float total = 0.0f;
    for(float accuracy : prediction_accuracy_history_)
    {
        total += accuracy;
    }
    return total / prediction_accuracy_history_.size();
}

void InputPreprocessor::set_sampling_reference(int32_t step_pos, uint8_t signal_val, int32_t encoder_unit_pos)
{
    current_reference_.step_position = step_pos;
    current_reference_.signal_value = signal_val;
    current_reference_.encoder_unit_position = encoder_unit_pos;

    current_step_position_ = step_pos;
    last_sensor_signal_ = signal_val;
    steps_since_last_edge_ = 0;

    LOGI("Sampling reference set: step=%d, signal=%d, unit=%d",
           (int)step_pos, (int)signal_val, (int)encoder_unit_pos);
}

void InputPreprocessor::reset()
{
    current_state_ = ProcessingState::INITIALIZING;
    current_direction_ = StepDirection::FORWARD;
    last_direction_ = StepDirection::FORWARD;

    current_step_position_ = 0;
    steps_since_last_edge_ = 0;
    backlash_compensation_steps_ = 0;
    last_sensor_signal_ = 0;

    current_reference_ = SamplingReference();
    predicted_next_reference_ = SamplingReference();

    prediction_accuracy_history_.clear();
    total_edges_detected_ = 0;
    successful_predictions_ = 0;
    direction_changes_ = 0;

    LOGD("InputPreprocessor reset completed");
}

void InputPreprocessor::update_config(const EdgeSamplingConfig &config)
{
    config_ = config;

    LOGI("Config updated: steps_per_unit=%.2f, backlash=%d",
            config_.steps_per_encoder_unit, config_.backlash_compensation);
}

} // namespace AbsoluteEncoder
