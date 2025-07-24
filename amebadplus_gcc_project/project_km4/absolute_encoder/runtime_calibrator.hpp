#ifndef RUNTIME_CALIBRATOR_HPP
#define RUNTIME_CALIBRATOR_HPP

#include <cstdint>
#include <deque>
#include <vector>
#include <cmath>

namespace AbsoluteEncoder
{

/**
 * 运行时校准器 - 基于原始数据的步数校准
 * 
 * 功能：
 * - 接收每步的原始传感器数据
 * - 检测信号变化并统计实际步数
 * - 校准步数/单位比值
 */
class RuntimeCalibrator
{
public:
    /**
     * 校准配置参数
     */
    struct CalibrationConfig
    {
        float nominal_steps_per_unit;           // 标称步数每单位
        float tolerance_ratio;                  // 允许偏差比例 (0.0-1.0)
        uint16_t min_samples_for_calibration;   // 校准所需最小样本数
        uint16_t max_samples_for_analysis;      // 分析缓存最大样本数

        // 默认配置
        CalibrationConfig() :
            nominal_steps_per_unit(28.8f),
            tolerance_ratio(0.15f),
            min_samples_for_calibration(10),
            max_samples_for_analysis(30) {}
    };

    /**
     * 校准状态
     */
    enum class CalibrationState
    {
        UNCALIBRATED,       // 未校准
        COLLECTING,         // 收集数据中
        CALIBRATED,         // 已校准
        FAILED              // 校准失败
    };    /**
     * 原始步进数据
     */
    struct RawStepData
    {
        uint8_t sensor_signal;              // 传感器信号 (0或1)

        RawStepData() :
            sensor_signal(0) {}
    };

    /**
     * 校准结果
     */
    struct CalibrationResult
    {
        CalibrationState state;
        float calibrated_steps_per_unit;     // 校准后的步数每单位
        float confidence_level;              // 置信度等级 (0.0-1.0)
        uint32_t sample_count;               // 使用的样本数
        float step_variance;                 // 步数方差

        CalibrationResult() :
            state(CalibrationState::UNCALIBRATED),
            calibrated_steps_per_unit(0),
            confidence_level(0.0f),
            sample_count(0),
            step_variance(0.0f) {}
    };

public:
    /**
     * 构造函数
     * @param config 校准配置
     */
    explicit RuntimeCalibrator(const CalibrationConfig &config);

    /**
     * 添加原始步进数据
     * @param data 步进数据
     */
    void add_raw_step_data(const RawStepData &data);

    /**
     * 获取当前校准结果
     * @return 校准结果
     */
    CalibrationResult get_calibration_result() const;

    /**
     * 强制执行校准分析
     * @return 是否分析成功
     */
    bool force_analyze();

    /**
     * 重置校准器
     */
    void reset();

    /**
     * 更新配置
     * @param config 新配置
     */
    void update_config(const CalibrationConfig &config);

    /**
     * 获取当前状态
     */
    CalibrationState get_state() const
    {
        return current_state_;
    }

    /**
     * 获取收集的样本数量
     */
    uint32_t get_sample_count() const
    {
        return step_samples_.size();
    }

    /**
     * 检查是否需要更新校准
     * @return 是否需要重新校准
     */
    bool needs_recalibration() const;

private:
    // 配置参数
    CalibrationConfig config_;

    // 当前状态
    CalibrationState current_state_;
    CalibrationResult last_result_;

    // 步数样本数据
    std::deque<float> step_samples_;    // 步数跟踪（用于原始数据处理）
    uint16_t current_step_count_;           // 当前编码器单位的累计步数
    uint8_t last_sensor_signal_;            // 上次的传感器信号
    bool signal_initialized_;               // 信号是否已初始化

    // 分析相关
    uint16_t last_analysis_sample_count_;

    // ============ 核心分析方法 ============

    /**
     * 执行数据分析
     * @return 是否分析成功
     */
    bool perform_calibration_analysis();

    /**
     * 分析步数数据
     * @param mean_steps 输出：平均步数
     * @param variance 输出：方差
     * @return 是否分析成功
     */
    bool analyze_step_data(float &mean_steps, float &variance);

    /**
     * 计算置信度
     * @param variance 数据方差
     * @param sample_count 样本数量
     * @return 置信度 (0.0-1.0)
     */
    float calculate_confidence_level(float variance, uint32_t sample_count);

    /**
     * 验证校准结果的合理性
     * @param result 校准结果
     * @return 是否通过验证
     */
    bool validate_calibration_result(const CalibrationResult &result);

    /**
     * 尝试自动分析
     */
    void try_automatic_analysis();    /**
     * 检测编码器信号变化并记录单位完成
     * @param new_signal 新的传感器信号
     */
    void detect_signal_transition(uint8_t new_signal);

    /**
     * 重置步数跟踪状态
     */
    void reset_step_tracking();
};

} // namespace AbsoluteEncoder

#endif // RUNTIME_CALIBRATOR_HPP
