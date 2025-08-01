#include "absolute_encoder.h"
#include "position_tracker.hpp"
#include <algorithm>
#include <cmath>

namespace AbsoluteEncoder
{

PositionTracker::PositionTracker(EncoderMap &encoder_map, const uint8_t *pattern, uint16_t pattern_length, bool ring_mode)
    : encoder_map_(encoder_map),
        pattern_(pattern), pattern_length_(pattern_length),
        state_(TrackerState::UNINITIALIZED), current_position_(0),
        current_direction_(SearchDirection::FORWARD), consecutive_failures_(0),
        ring_mode_(ring_mode)
{
    clear_stats();

    LOGD("PositionTracker initialized: pattern_length=%d", pattern_length_);
}

void PositionTracker::initialize_position(uint16_t position, SearchDirection direction)
{
    current_position_ = position;
    current_direction_ = direction;
    state_ = TrackerState::TRACKING;
    consecutive_failures_ = 0;

    LOGD("Position tracker initialized: position=%d, direction=%s",
            position, (direction == SearchDirection::FORWARD) ? "FORWARD" : "BACKWARD");
}

ValidationResult PositionTracker::track_position(uint8_t bit, SearchDirection direction, uint16_t &tracked_position)
{
    if(state_ == TrackerState::UNINITIALIZED)
    {
        LOGE("Tracker not initialized");
        return ValidationResult::INVALID;
    }

    stats_.total_updates++;
    current_direction_ = direction;

    // 计算下一个预期位置和事件
    uint16_t expected_position = next_position(current_position_, direction, pattern_length_);
    uint8_t expected_bit = pattern_[expected_position];
    TrackerEvent event = determine_event(expected_bit, bit);

    // 状态机处理
    return handle_event(event, bit, direction, tracked_position);
}

// ============ 状态机核心实现 ============

ValidationResult PositionTracker::handle_event(TrackerEvent event, uint8_t bit, SearchDirection direction, uint16_t &position)
{
    switch(state_)
    {
    case TrackerState::UNINITIALIZED:
        // 只能通过initialize_position进入跟踪状态
        return ValidationResult::INVALID;

    case TrackerState::TRACKING:
        return handle_tracking_event(event, bit, direction, position);

    case TrackerState::LOST:
        return handle_lost_event(event, bit, direction, position);
    }

    return ValidationResult::INVALID;
}

void PositionTracker::reset()
{
    state_ = TrackerState::UNINITIALIZED;
    current_position_ = 0;
    current_direction_ = SearchDirection::FORWARD;
    consecutive_failures_ = 0;

    LOGD("Position tracker reset");
}

void PositionTracker::clear_stats()
{
    stats_.total_updates = 0;
    stats_.successful_tracks = 0;
    stats_.validation_failures = 0;
    stats_.direction_changes = 0;
}

// ============ 事件处理 ============

TrackerEvent PositionTracker::determine_event(uint8_t expected_bit, uint8_t actual_bit)
{
    return (expected_bit == actual_bit) ? TrackerEvent::MATCH : TrackerEvent::MISMATCH;
}

ValidationResult PositionTracker::handle_tracking_event(TrackerEvent event, uint8_t bit, SearchDirection direction, uint16_t &position)
{
    switch(event)
    {
    case TrackerEvent::MATCH:
        current_position_ = next_position(current_position_, direction, pattern_length_);
        position = current_position_;
        consecutive_failures_ = 0;
        stats_.successful_tracks++;
        return ValidationResult::VALID;

    case TrackerEvent::MISMATCH:
        consecutive_failures_++;
        stats_.validation_failures++;
        position = current_position_;

        if(consecutive_failures_ >= MAX_CONSECUTIVE_FAILURES)
        {
            transition_to_state(TrackerState::LOST);
            return ValidationResult::INVALID;
        }
        return ValidationResult::INVALID;

    default:
        return ValidationResult::INVALID;
    }
}

ValidationResult PositionTracker::handle_lost_event(TrackerEvent event, uint8_t bit, SearchDirection direction, uint16_t &position)
{
    // 在丢失状态时，只能等待外部重新初始化
    position = current_position_;
    return ValidationResult::INVALID;
}

void PositionTracker::transition_to_state(TrackerState new_state)
{
    if(state_ != new_state)
    {
        LOGD("State transition: %d -> %d", (int)state_, (int)new_state);
        state_ = new_state;
    }
}

// ============ 位置计算模块 ============

uint16_t PositionTracker::next_position(uint16_t current, SearchDirection direction, uint16_t pattern_length)
{
    if(ring_mode_)
    {
        if(direction == SearchDirection::FORWARD)
        {
            return (current + 1) % pattern_length;
        }
        else
        {
            return (current == 0) ? (pattern_length - 1) : (current - 1);
        }
    }
    else
    {
        if(direction == SearchDirection::FORWARD)
        {
            return (current + 1 < pattern_length) ? (current + 1) : current;
        }
        else
        {
            return (current > 0) ? (current - 1) : current;
        }
    }
}

int PositionTracker::calculate_distance(uint16_t pos1, uint16_t pos2, uint16_t pattern_length)
{
    int distance = std::abs((int)pos1 - (int)pos2);
    return std::min(distance, (int)pattern_length - distance);
}

} // namespace AbsoluteEncoder
