#include <cmath>
#include <algorithm>
#include <vector>

#include "arm_math.h"

#include "absolute_encoder.h"
#include "runtime_calibrator.hpp"

namespace AbsoluteEncoder
{

// ============ 构造函数和基础管理 ============

RuntimeCalibrator::RuntimeCalibrator(const CalibrationConfig &config)
    : config_(config),
      current_state_(CalibrationState::UNCALIBRATED),
      current_step_count_(0),
      last_sensor_signal_(0),
      signal_initialized_(false),
      last_analysis_sample_count_(0)
      
{
    // 初始化默认校准结果
    last_result_.state = CalibrationState::UNCALIBRATED;
    last_result_.calibrated_steps_per_unit = config_.nominal_steps_per_unit;
    last_result_.confidence_level = 0.0f;

    // 初始化环形缓冲区
    step_samples_.reserve(config_.max_samples_for_analysis);
    head_index_ = 0;
}

void RuntimeCalibrator::reset()
{
    head_index_ = 0;
    step_samples_.clear();
    current_state_ = CalibrationState::UNCALIBRATED;
    last_analysis_sample_count_ = 0;
    
    // 重置步数跟踪
    reset_step_tracking();

    // 重置结果为默认值
    last_result_.state = CalibrationState::UNCALIBRATED;
    last_result_.calibrated_steps_per_unit = config_.nominal_steps_per_unit;
    last_result_.confidence_level = 0.0f;
}

void RuntimeCalibrator::update_config(const CalibrationConfig &config)
{
    bool needs_reset = (config.nominal_steps_per_unit != config_.nominal_steps_per_unit);
    config_ = config;

    if(needs_reset)
    {
        reset();
    }
}

// ============ 数据收集接口 ============

void RuntimeCalibrator::add_raw_step_data(const RawStepData &data)
{
    // 步数计数递增
    current_step_count_++;
    
    // 检测信号变化
    detect_signal_transition(data.sensor_signal);
}

RuntimeCalibrator::CalibrationResult RuntimeCalibrator::get_calibration_result() const
{
    return last_result_;
}

bool RuntimeCalibrator::needs_recalibration() const
{
    if(last_result_.state != CalibrationState::CALIBRATED)
    {
        return true;
    }
    
    // 检查是否有足够新数据需要重新分析
    return step_samples_.size() >= last_analysis_sample_count_ + config_.min_samples_for_calibration;
}

// ============ 核心分析方法 ============

bool RuntimeCalibrator::perform_calibration_analysis()
{
    if(step_samples_.size() < config_.min_samples_for_calibration)
    {
        return false;
    }
    
    last_analysis_sample_count_ = step_samples_.size();
    
    float mean_steps, step_variance;
    if(!analyze_step_data(mean_steps, step_variance))
    {
        current_state_ = CalibrationState::FAILED;
        return false;
    }
    
    // 创建校准结果
    CalibrationResult new_result;
    new_result.state = CalibrationState::CALIBRATED;
    new_result.calibrated_steps_per_unit = mean_steps;
    new_result.step_variance = step_variance;
    new_result.sample_count = step_samples_.size();
    
    // 计算置信度
    new_result.confidence_level = calculate_confidence_level(step_variance, step_samples_.size());
    LOGI("Calibration result in %d samples: steps_per_unit=%.2f, variance=%.2f, confidence=%.2f",
        (int)new_result.sample_count, new_result.calibrated_steps_per_unit, step_variance, new_result.confidence_level);

    // 验证结果
    if(validate_calibration_result(new_result))
    {
        last_result_ = new_result;
        LOGI("Accepted calibration result");
        current_state_ = CalibrationState::CALIBRATED;
        return true;
    }
    else
    {
        LOGI("Denied calibration result");
        current_state_ = CalibrationState::FAILED;
        return false;
    }
}

bool RuntimeCalibrator::analyze_step_data(float &mean_steps, float &variance)
{
    if(step_samples_.empty())
    {
        return false;
    }

    // 使用 CMSIS-DSP 函数
    arm_mean_f32(step_samples_.data(), step_samples_.size(), &mean_steps);
    arm_var_f32(step_samples_.data(), step_samples_.size(), &variance);
    
    return true;
}

// ============ 校准计算方法 ============

float RuntimeCalibrator::calculate_confidence_level(float variance, uint32_t sample_count)
{
    // 基于方差计算基础置信度
    float variance_confidence = 1.0f / (1.0f + variance / config_.nominal_steps_per_unit);

    // 基于样本数计算样本置信度
    float sample_confidence = std::min(1.0f, static_cast<float>(sample_count) /
                                        (config_.min_samples_for_calibration * 2));

    // 综合置信度（加权平均）
    float confidence = (variance_confidence * 0.6f + sample_confidence * 0.4f);

    return std::min(1.0f, std::max(0.0f, confidence));
}

bool RuntimeCalibrator::validate_calibration_result(const CalibrationResult &result)
{
    // 检查步数是否在合理范围内
    float tolerance = config_.nominal_steps_per_unit * config_.tolerance_ratio;
    if(std::abs(result.calibrated_steps_per_unit - config_.nominal_steps_per_unit) > tolerance)
    {
        return false;
    }
    
    // 检查置信度是否足够
    if(result.confidence_level < 0.7f)
    {
        return false;
    }
    
    return true;
}

void RuntimeCalibrator::try_automatic_analysis()
{
    // 检查是否有足够样本进行分析
    if(step_samples_.size() >= config_.min_samples_for_calibration && 
       step_samples_.size() != last_analysis_sample_count_)
    {
        perform_calibration_analysis();
    }
}

bool RuntimeCalibrator::force_analyze()
{
    if(step_samples_.size() < config_.min_samples_for_calibration)
    {
        return false;
    }

    return perform_calibration_analysis();
}

void RuntimeCalibrator::add_step_sample(float steps_per_unit)
{
    if(step_samples_.size() < config_.max_samples_for_analysis)
    {
        step_samples_.push_back(steps_per_unit);
    }
    else
    {
        // 环形缓冲区已满，覆盖旧数据
        step_samples_[head_index_] = steps_per_unit;
        head_index_ = (head_index_ + 1) % config_.max_samples_for_analysis;
    }
}

void RuntimeCalibrator::detect_signal_transition(uint8_t new_signal)
{
    // 如果信号尚未初始化，记录初始信号
    if(!signal_initialized_)
    {
        last_sensor_signal_ = new_signal;
        signal_initialized_ = true;
        return;
    }
    
    // 检测信号变化（编码器单位完成）
    if(new_signal != last_sensor_signal_)
    {
        // 确保有足够的步数（过滤噪声）
        if(current_step_count_ >= 3)  // 最少3步才认为是有效的编码器单位
        {
            // 计算参考步数值和容差上限
            float reference_steps = (last_result_.state == CalibrationState::CALIBRATED) ? 
                                   last_result_.calibrated_steps_per_unit : 
                                   config_.nominal_steps_per_unit;
            
            float tolerance_upper_limit = reference_steps * (1.0f + config_.tolerance_ratio);
            
            // 如果步数超过容差上限，进行等分处理
            if(current_step_count_ > tolerance_upper_limit)
            {
                // 估算分割数和每段步数
                int devider = std::round(current_step_count_ / reference_steps);
                float steps_per_unit = current_step_count_ / (float)devider;
                
                for(uint32_t i = 0; i < (uint32_t)devider; i++)
                {
                    add_step_sample(steps_per_unit);
                }
            }
            // 否则直接添加当前步数样本
            else
            {
                add_step_sample((float)current_step_count_);
            }
            
            // 如果达到足够样本数，尝试自动分析
            try_automatic_analysis();
        }
        
        // 重置步数计数
        current_step_count_ = 0;
        last_sensor_signal_ = new_signal;
    }
}

void RuntimeCalibrator::reset_step_tracking()
{
    current_step_count_ = 0;
    last_sensor_signal_ = 0;
    signal_initialized_ = false;
}

} // namespace AbsoluteEncoder
