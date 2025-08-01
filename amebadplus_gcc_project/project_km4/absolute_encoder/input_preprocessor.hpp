#ifndef INPUT_PREPROCESSOR_HPP
#define INPUT_PREPROCESSOR_HPP

#include <cstdint>
#include <deque>
#include <vector>
#include "runtime_calibrator.hpp"

namespace AbsoluteEncoder
{

/**
 * 基于边沿预测的输入信号处理器
 * 采用类似串口解析的思想处理编码器信号：
 * - 基于信号边沿建立采样基准点
 * - 预测性边沿检测处理换向
 * - 齿轮回差补偿
 * - 自适应采样时机调整
 */
class InputPreprocessor
{
public:
    /**
     * 边沿采样配置参数
     */
    struct EdgeSamplingConfig
    {
        float steps_per_encoder_unit;       // 每个编码器单位的理论步数
        float sampling_point_ratio;         // 采样点位置比例 (0.0-1.0)，0.5表示在码盘刻度中点采样
        uint16_t backlash_compensation;     // 齿轮回差补偿步数
        uint16_t symertric_compensation;    // 对称补偿步数（用于遮光条纹边缘补偿）
        float edge_prediction_tolerance;    // 边沿预测容差 (0.0-1.0)
        bool enable_adaptive_prediction;    // 启用自适应边沿预测
        uint16_t min_steps_for_edge;        // 边沿检测最小步数间隔
        uint16_t max_steps_for_edge;        // 边沿检测最大步数间隔

        EdgeSamplingConfig() :
            steps_per_encoder_unit(28.8f),
            sampling_point_ratio(0.5f),     // 默认在码盘刻度中点采样
            backlash_compensation(0),
            symertric_compensation(0),
            edge_prediction_tolerance(0.15f),
            enable_adaptive_prediction(true),
            min_steps_for_edge(5),
            max_steps_for_edge(60) {}
    };

    /**
     * 信号处理状态
     */
    enum class ProcessingState
    {
        INITIALIZING,           // 初始化中，等待第一个边沿
        TRACKING_FORWARD,       // 正向跟踪中
        TRACKING_BACKWARD,      // 反向跟踪中
        BACKLASH_COMPENSATING,  // 齿轮回差补偿中
    };

    /**
     * 信号处理结果
     */
    struct ProcessingResult
    {
        bool should_sample_signal;          // 是否到达采样点（通知上层处理）
        bool unit_completed;                // 是否完成了一个编码器单位
        uint8_t current_signal_value;       // 当前信号值 (0或1)
        int32_t encoder_unit_change;        // 编码器单位变化
        ProcessingState new_state;          // 新的处理状态

        ProcessingResult() : should_sample_signal(false), unit_completed(false),
            current_signal_value(0), encoder_unit_change(0),
            new_state(ProcessingState::INITIALIZING) {}
    };

    /**
     * 采样基准点信息
     */
    struct SamplingReference
    {
        int32_t step_position;              // 基准点的步数位置
        uint8_t signal_value;               // 基准点的信号值
        int32_t encoder_unit_position;      // 基准点的编码器单位位置
        float prediction_error;             // 上次预测误差

        SamplingReference() : step_position(0), signal_value(0),
            encoder_unit_position(0), prediction_error(0.0f) {}
    };

public:
    /**
     * 步进方向
     */
    enum class StepDirection
    {
        FORWARD = 1,        // 前进
        BACKWARD = -1       // 后退
    };

    /**
     * 构造函数
     * @param config 边沿采样配置
     */
    InputPreprocessor(const EdgeSamplingConfig &config);

    /**
     * 处理电机步进和传感器信号（主要处理接口）
     * @param direction 步进方向
     * @param sensor_signal 当前传感器信号 (0或1)
     * @return 信号处理结果
     */
    ProcessingResult process_step_with_signal(StepDirection direction, uint8_t sensor_signal);

    /**
     * 获取当前处理状态
     */
    ProcessingState get_processing_state() const
    {
        return current_state_;
    }

    /**
     * 获取当前采样基准点信息
     */
    const SamplingReference &get_current_reference() const
    {
        return current_reference_;
    }

    /**
     * 手动设置采样基准点（用于初始化或校正）
     * @param step_pos 步数位置
     * @param signal_val 信号值
     * @param encoder_unit_pos 编码器单位位置
     */
    void set_sampling_reference(int32_t step_pos, uint8_t signal_val, int32_t encoder_unit_pos);
    
    /**
     * @brief 获取当前估算的子单位位置
     * @return 当前估算的子单位位置
     */
    float get_estimated_unit_offset();

    /**
     * 重置处理器状态
     */
    void reset();

    /**
     * 更新配置
     */
    void update_config(const EdgeSamplingConfig &config);
    const EdgeSamplingConfig &get_config() const
    {
        return config_;
    }

private:
    // 配置参数
    EdgeSamplingConfig config_;

    // 处理状态
    ProcessingState current_state_;
    StepDirection current_direction_;
    StepDirection last_direction_;

    // 采样基准管理
    SamplingReference current_reference_;
    SamplingReference predicted_next_reference_;

    // 步数跟踪
    int32_t current_step_position_;         // 当前步数位置
    int32_t steps_since_last_unit_;         // 距离上次边沿的步数
    int32_t backlash_compensation_steps_;   // 剩余回差补偿步数
    bool already_sampled_at_unit_;          // 是否已经在当前单位采样过

    // 边沿预测和检测
    uint8_t last_sensor_signal_;            // 上次传感器信号

    /**
     * 检测信号边沿
     * @param current_signal 当前信号值
     * @return 是否检测到边沿
     */
    bool detect_signal_edge(uint8_t current_signal);

    /**
     * 计算换向后使用的基准边沿位置
     * @param from_direction 当前方向
     * @param to_direction 目标方向
     * @return 换向后使用的基准边沿位置
     */
    int32_t calculate_based_edge_for_sample(StepDirection from_direction, StepDirection to_direction);

    /**
     * 处理方向变化
     * @param new_direction 新方向
     * @return 是否需要进行回差补偿
     */
    bool handle_direction_change(StepDirection new_direction);

    /**
     * 更新采样基准点
     * @param step_pos 新的步数位置
     * @param signal_val 新的信号值
     */
    void update_sampling_reference(int32_t step_pos, uint8_t signal_val);

    /**
     * 计算编码器单位变化
     * @return 编码器单位变化量
     */
    int32_t calculate_encoder_unit_change();

    /**
     * 检查是否应该触发边沿采样
     * @return 是否应该采样
     */
    bool should_trigger_edge_sampling();

    /**
     * 处理初始化状态
     * @param sensor_signal 传感器信号
     * @return 处理结果
     */
    ProcessingResult handle_initialization(uint8_t sensor_signal);

    /**
     * 处理跟踪状态
     * @param sensor_signal 传感器信号
     * @return 处理结果
     */
    ProcessingResult handle_tracking(uint8_t sensor_signal);

    /**
     * 处理回差补偿状态
     * @param sensor_signal 传感器信号
     * @return 处理结果
     */
    ProcessingResult handle_backlash_compensation(uint8_t sensor_signal);
};

} // namespace AbsoluteEncoder

#endif // INPUT_PREPROCESSOR_HPP