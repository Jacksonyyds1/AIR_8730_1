#ifndef POSITION_TRACKER_HPP
#define POSITION_TRACKER_HPP

#include "encoder_map.hpp"
#include <cstdint>
#include <deque>

namespace AbsoluteEncoder
{

// 前向声明
class InputPreprocessor;

/**
 * 位置跟踪器状态机状态
 */
enum class TrackerState
{
    UNINITIALIZED,  // 未初始化
    TRACKING,       // 正常跟踪
    LOST           // 跟踪丢失
};

/**
 * 状态机事件
 */
enum class TrackerEvent
{
    INITIALIZE,     // 初始化
    MATCH,          // 匹配
    MISMATCH,       // 不匹配
    RESET           // 重置
};

/**
 * 跟踪验证结果
 */
enum class ValidationResult
{
    VALID,         // 位置有效
    INVALID        // 位置无效
};

/**
 * 位置跟踪器
 * 简化的位置跟踪器，专注于基本的位置跟踪功能
 * 边缘检测和容差处理由 InputPreprocessor 负责
 */
class PositionTracker
{
public:
    /**
     * 构造函数
     * @param encoder_map 编码器映射对象引用
     * @param pattern 码盘图案数据
     * @param pattern_length 码盘图案长度
     * @param ring_mode 是否启用环形编码器模式
     */
    PositionTracker(EncoderMap &encoder_map, const uint8_t *pattern, uint16_t pattern_length, bool ring_mode);

    /**
     * 设置初始位置，进入跟踪模式
     * @param position 初始位置
     * @param direction 初始运动方向
     */
    void initialize_position(uint16_t position, SearchDirection direction);

    /**
     * 主要跟踪方法
     * @param bit 新输入的位值
     * @param direction 当前运动方向
     * @param tracked_position 输出参数：跟踪到的位置
     * @return 跟踪结果
     */
    ValidationResult track_position(uint8_t bit, SearchDirection direction, uint16_t &tracked_position);

    /**
     * 重置跟踪器状态
     */
    void reset();

    // 状态查询方法
    TrackerState get_state() const
    {
        return state_;
    }
    uint16_t get_current_position() const
    {
        return current_position_;
    }
    SearchDirection get_current_direction() const
    {
        return current_direction_;
    }
    bool is_ready() const
    {
        return state_ != TrackerState::UNINITIALIZED;
    }

    /**
     * 跟踪统计信息
     */
    struct TrackingStats
    {
        uint32_t total_updates;             // 总更新次数
        uint32_t successful_tracks;         // 成功跟踪次数
        uint32_t validation_failures;       // 验证失败次数
        uint32_t direction_changes;         // 方向改变次数
    };

    const TrackingStats &get_stats() const
    {
        return stats_;
    }
    void clear_stats();

private:
    // 核心组件引用
    EncoderMap &encoder_map_;
    const uint8_t *pattern_;
    uint16_t pattern_length_;
    bool ring_mode_;            // 是否启用环形编码器模式

    // 状态机核心
    TrackerState state_;
    uint16_t current_position_;
    SearchDirection current_direction_;
    uint8_t consecutive_failures_;
    static const uint8_t MAX_CONSECUTIVE_FAILURES = 3;

    // 统计信息
    TrackingStats stats_;

    // ============ 状态机核心方法 ============

    /**
     * 状态机事件处理器
     */
    ValidationResult handle_event(TrackerEvent event, uint8_t bit, SearchDirection direction, uint16_t &position);

    /**
     * 状态处理方法
     */
    ValidationResult handle_tracking_event(TrackerEvent event, uint8_t bit, SearchDirection direction, uint16_t &position);
    ValidationResult handle_lost_event(TrackerEvent event, uint8_t bit, SearchDirection direction, uint16_t &position);

    /**
     * 状态转换方法
     */
    void transition_to_state(TrackerState new_state);

    /**
     * 确定下一个事件
     */
    TrackerEvent determine_event(uint8_t expected_bit, uint8_t actual_bit);

    // ============ 位置计算模块 ============

    
    /**
     * @brief 计算下一个位置
     * @param current 当前位置信息
     * @param direction 当前运动方向
     * @param pattern_length 图案长度
     * @return 下一个位置
     */
    uint16_t next_position(uint16_t current, SearchDirection direction, uint16_t pattern_length);

    /**
     * 计算两个位置之间的距离
     * @param pos1 第一个位置
     * @param pos2 第二个位置
     * @param pattern_length 图案长度
     * @return 两个位置之间的最小距离
     */
    int calculate_distance(uint16_t pos1, uint16_t pos2, uint16_t pattern_length);
};

} // namespace AbsoluteEncoder

#endif // POSITION_TRACKER_HPP