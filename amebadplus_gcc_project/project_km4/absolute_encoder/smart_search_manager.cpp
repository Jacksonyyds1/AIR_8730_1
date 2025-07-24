#include "absolute_encoder.h"
#include "smart_search_manager.hpp"
#include <algorithm>

namespace AbsoluteEncoder
{

SmartSearchManager::SmartSearchManager(EncoderMap &encoder_map, uint8_t max_buffer_size)
    : encoder_map_(encoder_map), max_buffer_size_(max_buffer_size),
        current_start_offset_(0), current_search_node_(nullptr),
        current_direction_(SearchDirection::FORWARD), last_processed_index_(0)
{

    buffer_.reserve(max_buffer_size_);
    clear_stats();

    LOGD("SmartSearchManager initialized with max_buffer_size=%d", max_buffer_size_);
}

SearchResult SmartSearchManager::add_bit_and_search(uint8_t bit, SearchDirection direction, uint16_t &position)
{
    uint8_t sequence_length, start_offset;
    return add_bit_and_search(bit, direction, position, sequence_length, start_offset);
}

SearchResult SmartSearchManager::add_bit_and_search(uint8_t bit, SearchDirection direction, uint16_t &position,
        uint8_t &sequence_length, uint8_t &start_offset)
{
    // 添加新位到缓冲区
    add_to_buffer(bit);

    // 如果方向改变，重置搜索状态
    if(direction != current_direction_)
    {
        current_direction_ = direction;
        current_start_offset_ = 0;
        current_search_node_ = nullptr;
        LOGD("Search direction changed, resetting search state");
    }

    // 从当前偏移开始搜索
    SearchResult result = search_from_offset(current_start_offset_, direction, position);

    // 如果搜索失败，尝试滑动窗口重试
    if(result == SearchResult::NOT_FOUND)
    {
        result = shift_and_retry(direction, position);
    }

    // 返回实际的匹配信息
    if(result == SearchResult::FOUND)
    {
        start_offset = current_start_offset_;
        // 使用找到的搜索节点的深度信息
        if(current_search_node_ != nullptr)
        {
            sequence_length = current_search_node_->depth;
        }
        else
        {
            sequence_length = buffer_.size() - current_start_offset_;
        }
    }
    else
    {
        start_offset = 0;
        sequence_length = 0;
    }

    // 更新统计信息
    update_stats(result, sequence_length);

    return result;
}

void SmartSearchManager::reset()
{
    buffer_.clear();
    current_start_offset_ = 0;
    current_search_node_ = nullptr;
    current_direction_ = SearchDirection::FORWARD;
    last_processed_index_ = 0;

    LOGD("SmartSearchManager reset");
}

void SmartSearchManager::clear_stats()
{
    stats_.total_searches = 0;
    stats_.successful_searches = 0;
    stats_.retry_count = 0;
    stats_.max_sequence_length = 0;
}

SearchResult SmartSearchManager::search_from_offset(uint8_t start_offset, SearchDirection direction, uint16_t &position)
{
    if(start_offset >= buffer_.size())
    {
        return SearchResult::NOT_FOUND;
    }

    stats_.total_searches++;

    // 如果是新的搜索起点或者当前节点为空，开始新搜索
    if(current_search_node_ == nullptr || start_offset != current_start_offset_)
    {
        current_start_offset_ = start_offset;
        last_processed_index_ = start_offset;
        uint8_t start_bit = buffer_[start_offset];
        current_search_node_ = encoder_map_.start_search(start_bit, direction);

        if(current_search_node_ == nullptr)
        {
            LOGV("Failed to start search with bit %d at offset %d", start_bit, start_offset);
            return SearchResult::NOT_FOUND;
        }

        LOGV("Started new search at offset %d with bit %d", start_offset, start_bit);

        // 如果只有一个位，检查是否已经匹配
        if(buffer_.size() == (size_t)start_offset + 1)
        {
            // 检查当前节点是否为叶子节点
            bool is_leaf = (direction == SearchDirection::FORWARD) ?
                            current_search_node_->is_forward_leaf : current_search_node_->is_backward_leaf;

            if(is_leaf)
            {
                position = (direction == SearchDirection::FORWARD) ?
                            current_search_node_->forward_position : current_search_node_->backward_position;
                stats_.successful_searches++;
                LOGV("Found match at offset %d with single bit", start_offset);
                return SearchResult::FOUND;
            }
        }
    }

    // 继续绝对搜索 - 只处理新添加的位
    for(uint8_t i = last_processed_index_ + 1; i < buffer_.size(); ++i)
    {
        uint8_t bit = buffer_[i];
        SearchResult result = encoder_map_.absolute_search(current_search_node_, bit, direction, position);

        LOGV("Absolute search at index %d, bit %d, result %d", i, bit, (int)result);

        if(result == SearchResult::FOUND)
        {
            stats_.successful_searches++;
            uint8_t sequence_length = i - start_offset + 1;
            if(sequence_length > stats_.max_sequence_length)
            {
                stats_.max_sequence_length = sequence_length;
            }
            LOGI("Search successful at offset %d, sequence length %d, position %d",
                    start_offset, sequence_length, position);

            // 添加详细的调试信息
            LOGD("Buffer content from offset %d: ", start_offset);
            for(uint8_t j = start_offset; j <= i; ++j)
            {
                LOGD("  [%d]: %d", j, buffer_[j]);
            }

            last_processed_index_ = i;
            return SearchResult::FOUND;
        }
        else if(result == SearchResult::NOT_FOUND)
        {
            LOGV("Search failed at index %d", i);
            return SearchResult::NOT_FOUND;
        }
        // result == SearchResult::PARTIAL，继续搜索
        last_processed_index_ = i;
    }

    // 搜索未完成，但还有部分匹配
    return SearchResult::PARTIAL;
}

SearchResult SmartSearchManager::shift_and_retry(SearchDirection direction, uint16_t &position)
{
    uint8_t original_offset = current_start_offset_;

    // 尝试从不同的起始偏移重新搜索
    for(uint8_t offset = original_offset + 1; offset < buffer_.size(); ++offset)
    {
        stats_.retry_count++;
        current_search_node_ = nullptr;  // 重置搜索节点

        SearchResult result = search_from_offset(offset, direction, position);

        if(result == SearchResult::FOUND)
        {
            LOGD("Retry successful at offset %d after %d retries",
                    offset, offset - original_offset);
            return SearchResult::FOUND;
        }
        else if(result == SearchResult::PARTIAL)
        {
            // 找到部分匹配，停止重试
            LOGV("Found partial match at offset %d", offset);
            return SearchResult::PARTIAL;
        }
    }

    // 所有重试都失败
    LOGV("All retries failed for buffer size %d", (int)buffer_.size());
    return SearchResult::NOT_FOUND;
}

void SmartSearchManager::add_to_buffer(uint8_t bit)
{
    buffer_.push_back(bit);

    // 如果缓冲区超过最大大小，移除最早的位
    if(buffer_.size() > max_buffer_size_)
    {
        buffer_.erase(buffer_.begin());

        // 调整当前偏移和处理索引
        if(current_start_offset_ > 0)
        {
            current_start_offset_--;
            last_processed_index_--;
        }
        else
        {
            // 如果当前搜索的起始点被移除，重置搜索状态
            current_search_node_ = nullptr;
            current_start_offset_ = 0;
            last_processed_index_ = 0;
            LOGV("Buffer overflow, reset search state");
        }
    }

    LOGV("Added bit %d to buffer, buffer size: %d", bit, (int)buffer_.size());
}

void SmartSearchManager::update_stats(SearchResult result, uint8_t sequence_length)
{
    if(result == SearchResult::FOUND && sequence_length > stats_.max_sequence_length)
    {
        stats_.max_sequence_length = sequence_length;
    }
}

} // namespace AbsoluteEncoder