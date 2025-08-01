#ifndef SMART_SEARCH_MANAGER_HPP
#define SMART_SEARCH_MANAGER_HPP

#include "encoder_map.hpp"
#include <vector>
#include <cstdint>

namespace AbsoluteEncoder
{

/**
 * 智能搜索管理器
 * 实现搜索失败后的自动重试机制，通过滑动窗口算法提高搜索成功率
 * 专注于搜索模式，不包含跟踪功能
 */
class SmartSearchManager
{
public:
    /**
     * 构造函数
     * @param encoder_map 编码器映射对象引用
     * @param max_buffer_size 最大缓冲区大小
     */
    SmartSearchManager(EncoderMap &encoder_map, uint8_t max_buffer_size = 32);

    /**
     * 添加新的输入位并执行智能搜索
     * @param bit 新输入的位值 (0 或 1)
     * @param direction 搜索方向
     * @param position 输出参数：找到匹配时的位置
     * @return 搜索结果状态
     */
    SearchResult add_bit_and_search(uint8_t bit, SearchDirection direction, uint16_t &position);

    /**
     * 添加新的输入位并执行智能搜索（带详细信息）
     * @param bit 新输入的位值 (0 或 1)
     * @param direction 搜索方向
     * @param position 输出参数：找到匹配时的位置
     * @param sequence_length 输出参数：匹配序列的长度
     * @param start_offset 输出参数：匹配序列在缓冲区中的起始偏移
     * @return 搜索结果状态
     */
    SearchResult add_bit_and_search(uint8_t bit, SearchDirection direction, uint16_t &position,
                                    uint8_t &sequence_length, uint8_t &start_offset);

    /**
     * 重置搜索状态，清空缓冲区
     */
    void reset();

    /**
     * 获取当前缓冲区大小
     */
    uint8_t get_buffer_size() const
    {
        return buffer_.size();
    }

    /**
     * 获取当前搜索起始偏移
     */
    uint8_t get_current_offset() const
    {
        return current_start_offset_;
    }

    /**
     * 获取搜索统计信息
     */
    struct SearchStats
    {
        uint32_t total_searches;        // 总搜索次数
        uint32_t successful_searches;   // 成功搜索次数
        uint32_t retry_count;          // 重试次数
        uint32_t max_sequence_length;  // 最大成功序列长度
    };

    const SearchStats &get_stats() const
    {
        return stats_;
    }

    /**
     * 清零统计信息
     */
    void clear_stats();

private:
    EncoderMap &encoder_map_;           // 编码器映射引用
    std::vector<uint8_t> buffer_;       // 输入位缓冲区
    uint8_t max_buffer_size_;           // 最大缓冲区大小

    // 当前搜索状态
    uint8_t current_start_offset_;      // 当前搜索起始偏移
    TreeNode *current_search_node_;     // 当前搜索节点
    SearchDirection current_direction_; // 当前搜索方向
    uint16_t last_processed_index_;     // 上次处理的缓冲区索引
    SearchStats stats_;                 // 搜索统计信息

    /**
     * 从指定偏移开始执行搜索
     * @param start_offset 起始偏移
     * @param direction 搜索方向
     * @param position 输出参数：找到匹配时的位置
     * @return 搜索结果状态
     */
    SearchResult search_from_offset(uint8_t start_offset, SearchDirection direction, uint16_t &position);

    /**
     * 滑动窗口并重试搜索
     * @param direction 搜索方向
     * @param position 输出参数：找到匹配时的位置
     * @return 搜索结果状态
     */
    SearchResult shift_and_retry(SearchDirection direction, uint16_t &position);

    /**
     * 添加新位到缓冲区
     * @param bit 新位值
     */
    void add_to_buffer(uint8_t bit);

    /**
     * 更新搜索统计信息
     * @param result 搜索结果
     * @param sequence_length 搜索序列长度
     */
    void update_stats(SearchResult result, uint8_t sequence_length);
};

} // namespace AbsoluteEncoder

#endif // SMART_SEARCH_MANAGER_HPP