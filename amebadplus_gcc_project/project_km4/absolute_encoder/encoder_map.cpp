#include "absolute_encoder.h"
#include "encoder_map.hpp"
#include <algorithm>
#include <vector>
#include <iostream>
#include <iomanip>
#include <queue>

namespace AbsoluteEncoder
{

EncoderMap::EncoderMap(const uint8_t *pattern, uint16_t pattern_length, uint8_t max_search_depth)
    : pattern_(pattern), pattern_length_(pattern_length), max_search_depth_(max_search_depth),
        root_(nullptr), use_pool_(false), node_pool_(nullptr),
        pool_size_(0), used_nodes_(0)
{

    if(pattern == nullptr || pattern_length == 0 || max_search_depth == 0)
    {
        LOGE("Invalid parameters: pattern=%p, length=%d, max_depth=%d",
                pattern, pattern_length, max_search_depth);
        return; // 无效参数
    }

    LOGI("EncoderMap constructor: pattern_length=%d, max_search_depth=%d",
            pattern_length, max_search_depth);
    build_search_tree();
}

EncoderMap::EncoderMap(const uint8_t *pattern, uint16_t pattern_length, uint8_t max_search_depth,
                        TreeNode *node_pool, uint16_t pool_size)
    : pattern_(pattern), pattern_length_(pattern_length), max_search_depth_(max_search_depth),
        root_(nullptr), use_pool_(true), node_pool_(node_pool),
        pool_size_(pool_size), used_nodes_(0)
{

    if(pattern == nullptr || pattern_length == 0 || max_search_depth == 0 ||
            node_pool == nullptr || pool_size == 0)
    {
        LOGE("Invalid pool parameters: pattern=%p, length=%d, max_depth=%d, pool=%p, pool_size=%d",
                pattern, pattern_length, max_search_depth, node_pool, pool_size);
        return; // 无效参数
    }

    LOGI("EncoderMap pool constructor: pattern_length=%d, max_search_depth=%d, pool_size=%d",
            pattern_length, max_search_depth, pool_size);

    // 初始化内存池
    // for(uint16_t i = 0; i < pool_size_; ++i)
    // {
    //     node_pool_[i] = TreeNode();
    // }

    build_search_tree();
}

EncoderMap::~EncoderMap()
{
    if(!use_pool_)
    {
        // 动态分配模式才需要释放内存
        destroy_tree(root_);
    }
    // 内存池模式不需要释放，由外部管理
}

void EncoderMap::build_search_tree()
{
    // 创建根节点
    root_ = allocate_node();

    if(root_ == nullptr)
    {
        LOGE("Failed to allocate root node");
        return; // 内存分配失败
    }

    root_->depth = 0;  // 设置根节点深度为0

    LOGI("Starting to build search tree");
    // 先构建正向序列，再在同一棵树上添加反向信息
    build_forward_sequences();
    build_backward_sequences();
    LOGI("Search tree construction completed");
}

void EncoderMap::build_forward_sequences()
{
    LOGD("Building forward sequences");

    // 记录已经被添加到树中的位置
    std::vector<bool> position_covered(pattern_length_, false);
    uint16_t covered_count = 0;

    // 从长度1开始，逐步增加长度寻找唯一子图
    for(uint8_t length = 1; length <= max_search_depth_ && covered_count < pattern_length_; ++length)
    {
        LOGV("Checking subsequences of length %d", length);

        uint16_t added_this_length = 0;

        // 检查每个位置的子图
        for(uint16_t start_pos = 0; start_pos < pattern_length_; ++start_pos)
        {
            // 计算该起始位置对应的目标位置
            uint16_t target_pos = (start_pos + length - 1) % pattern_length_;

            // 如果目标位置已经被更短的子图覆盖，跳过
            if(position_covered[target_pos])
            {
                continue;
            }

            // 检查该子图是否唯一
            if(is_subsequence_unique(start_pos, length, SearchDirection::FORWARD))
            {
                // 检查是否有更短的唯一子图是其子图
                if(!has_shorter_unique_subsequence(start_pos, length, SearchDirection::FORWARD))
                {
                    // 插入到搜索树
                    insert_unique_subsequence(start_pos, length, SearchDirection::FORWARD);

                    // 标记目标位置（搜索完成后编码器的位置）而不是起始位置
                    uint16_t target_pos = (start_pos + length - 1) % pattern_length_;
                    if(!position_covered[target_pos])
                    {
                        position_covered[target_pos] = true;
                        covered_count++;
                    }
                    added_this_length++;

                    LOGV("Added forward subsequence: start=%d, length=%d, target_pos=%d", start_pos, length, target_pos);
                }
            }
        }

        LOGV("Added %d subsequences of length %d", added_this_length, length);

        // 如果某个长度没有添加任何子图，说明所有位置都被更短的子图覆盖
        if(added_this_length == 0 && covered_count == pattern_length_)
        {
            LOGD("All positions covered by shorter subsequences, stopping at length %d", length);
            break;
        }
    }

    LOGD("Forward tree construction completed, covered %d/%d positions", covered_count, pattern_length_);
}

void EncoderMap::build_backward_sequences()
{
    LOGD("Building backward sequences");

    // 记录已经被添加到树中的位置
    std::vector<bool> position_covered(pattern_length_, false);
    uint16_t covered_count = 0;

    // 从长度1开始，逐步增加长度寻找唯一子图
    for(uint8_t length = 1; length <= max_search_depth_ && covered_count < pattern_length_; ++length)
    {
        LOGV("Checking backward subsequences of length %d", length);

        uint16_t added_this_length = 0;

        // 检查每个位置的反向子图
        for(uint16_t start_pos = 0; start_pos < pattern_length_; ++start_pos)
        {
            // 计算该起始位置对应的目标位置（反向）
            uint16_t target_pos = (start_pos - length + 1 + pattern_length_) % pattern_length_;

            // 如果目标位置已经被更短的子图覆盖，跳过
            if(position_covered[target_pos])
            {
                continue;
            }

            // 检查该反向子图是否唯一
            if(is_subsequence_unique(start_pos, length, SearchDirection::BACKWARD))
            {
                // 检查是否有更短的唯一子图是其子图
                if(!has_shorter_unique_subsequence(start_pos, length, SearchDirection::BACKWARD))
                {
                    // 插入到搜索树
                    insert_unique_subsequence(start_pos, length, SearchDirection::BACKWARD);

                    // 标记目标位置（反向搜索完成后编码器的位置）
                    uint16_t target_pos = (start_pos - length + 1 + pattern_length_) % pattern_length_;
                    if(!position_covered[target_pos])
                    {
                        position_covered[target_pos] = true;
                        covered_count++;
                    }
                    added_this_length++;

                    LOGV("Added backward subsequence: start=%d, length=%d, target_pos=%d", start_pos, length, target_pos);
                }
            }
        }

        LOGV("Added %d backward subsequences of length %d", added_this_length, length);

        // 如果某个长度没有添加任何子图，说明所有位置都被更短的子图覆盖
        if(added_this_length == 0 && covered_count == pattern_length_)
        {
            LOGD("All positions covered by shorter subsequences, stopping at length %d", length);
            break;
        }
    }

    LOGD("Backward tree construction completed, covered %d/%d positions", covered_count, pattern_length_);
}

bool EncoderMap::is_subsequence_unique(uint16_t start_pos, uint8_t length, SearchDirection direction) const
{
    // 检查从start_pos开始长度为length的子图是否在码盘上唯一
    for(uint16_t other_pos = 0; other_pos < pattern_length_; ++other_pos)
    {
        if(other_pos == start_pos)
        {
            continue;
        }

        bool matches = true;
        for(uint8_t i = 0; i < length; ++i)
        {
            uint16_t pos1, pos2;

            if(direction == SearchDirection::FORWARD)
            {
                pos1 = (start_pos + i) % pattern_length_;
                pos2 = (other_pos + i) % pattern_length_;
            }
            else
            {
                pos1 = (start_pos - i + pattern_length_) % pattern_length_;
                pos2 = (other_pos - i + pattern_length_) % pattern_length_;
            }

            if(pattern_[pos1] != pattern_[pos2])
            {
                matches = false;
                break;
            }
        }

        if(matches)
        {
            return false; // 找到重复，不唯一
        }
    }

    return true; // 唯一
}

bool EncoderMap::has_shorter_unique_subsequence(uint16_t start_pos, uint8_t length, SearchDirection direction) const
{
    // 检查是否存在更短的唯一子图，且该短子图的整个序列处于当前序列的范围内
    for(uint8_t shorter_len = 1; shorter_len < length; ++shorter_len)
    {
        // 检查所有可能的子图位置
        for(uint8_t offset = 0; offset <= length - shorter_len; ++offset)
        {
            uint16_t sub_start;

            if(direction == SearchDirection::FORWARD)
            {
                sub_start = (start_pos + offset) % pattern_length_;
            }
            else
            {
                sub_start = (start_pos - offset + pattern_length_) % pattern_length_;
            }

            // 检查短子图是否唯一
            if(is_subsequence_unique(sub_start, shorter_len, direction))
            {
                // 只需要检查短子图的首尾位置是否都在当前序列的首尾位置区间内
                uint16_t sub_end;
                uint16_t current_start = start_pos;
                uint16_t current_end;

                if(direction == SearchDirection::FORWARD)
                {
                    sub_end = (sub_start + shorter_len - 1) % pattern_length_;
                    current_end = (start_pos + length - 1) % pattern_length_;
                }
                else
                {
                    sub_end = (sub_start - shorter_len + 1 + pattern_length_) % pattern_length_;
                    current_end = (start_pos - length + 1 + pattern_length_) % pattern_length_;
                }

                // 检查短子图的首尾位置是否都在当前序列的区间内
                bool is_contained = false;

                if(direction == SearchDirection::FORWARD)
                {
                    // 正向：检查 [sub_start, sub_end] 是否在 [current_start, current_end] 内
                    if(current_start <= current_end)
                    {
                        // 非跨越边界情况
                        is_contained = (sub_start >= current_start && sub_end <= current_end);
                    }
                    else
                    {
                        // 跨越边界情况
                        is_contained = ((sub_start >= current_start || sub_start <= current_end) &&
                                        (sub_end >= current_start || sub_end <= current_end));
                    }
                }
                else
                {
                    // 反向：检查 [sub_end, sub_start] 是否在 [current_end, current_start] 内
                    if(current_end <= current_start)
                    {
                        // 非跨越边界情况
                        is_contained = (sub_end >= current_end && sub_start <= current_start);
                    }
                    else
                    {
                        // 跨越边界情况
                        is_contained = ((sub_end >= current_end || sub_end <= current_start) &&
                                        (sub_start >= current_end || sub_start <= current_start));
                    }
                }

                if(is_contained)
                {
                    return true; // 找到更短的唯一子图且完全包含在当前序列中
                }
            }
        }
    }

    return false; // 没有找到包含的更短唯一子图
}

void EncoderMap::insert_unique_subsequence(uint16_t start_pos, uint8_t length, SearchDirection direction)
{
    TreeNode *current = root_;

    // 沿着子图的位值路径在树中创建节点
    for(uint8_t i = 0; i < length; ++i)
    {
        uint16_t bit_pos;

        if(direction == SearchDirection::FORWARD)
        {
            bit_pos = (start_pos + i) % pattern_length_;
        }
        else
        {
            bit_pos = (start_pos - i + pattern_length_) % pattern_length_;
        }

        uint8_t bit = pattern_[bit_pos];
        TreeNode **next_node = (bit == 0) ? &(current->left) : &(current->right);

        if(*next_node == nullptr)
        {
            *next_node = allocate_node();
            if(*next_node == nullptr)
            {
                LOGE("Failed to allocate node for subsequence insertion");
                return;
            }
            (*next_node)->bit_value = bit;
            (*next_node)->parent = current;
            (*next_node)->depth = i + 1;  // 设置深度信息
        }

        current = *next_node;
    }

    // 在最后一个节点设置叶子信息
    if(direction == SearchDirection::FORWARD)
    {
        current->is_forward_leaf = true;
        // 正向搜索：当前位置 = 起始位置 + 子图长度 - 1
        current->forward_position = (start_pos + length - 1) % pattern_length_;
    }
    else
    {
        current->is_backward_leaf = true;
        // 反向搜索：当前位置 = 起始位置 - 子图长度 + 1
        current->backward_position = (start_pos - length + 1 + pattern_length_) % pattern_length_;
    }

    LOGV("Inserted %s subsequence: start=%d, length=%d, end_pos=%d",
            direction == SearchDirection::FORWARD ? "forward" : "backward",
            start_pos, length,
            direction == SearchDirection::FORWARD ? current->forward_position : current->backward_position);
}

TreeNode *EncoderMap::start_search(uint8_t bit, SearchDirection direction)
{
    // 避免未使用参数警告
    (void)direction;

    if(root_ == nullptr)
    {
        return nullptr;
    }

    // 从根节点开始搜索对应的位值
    TreeNode *next = (bit == 0) ? root_->left : root_->right;
    return next;
}

SearchResult EncoderMap::absolute_search(TreeNode *&current_node, uint8_t bit,
        SearchDirection direction, uint16_t &position)
{
    if(current_node == nullptr)
    {
        return SearchResult::NOT_FOUND;
    }

    // 首先处理新输入的bit，移动到下一个节点
    TreeNode *next = (bit == 0) ? current_node->left : current_node->right;

    if(next == nullptr)
    {
        return SearchResult::NOT_FOUND;
    }

    current_node = next;

    // 检查新节点是否为叶子节点
    bool is_leaf = (direction == SearchDirection::FORWARD) ?
                    current_node->is_forward_leaf : current_node->is_backward_leaf;

    if(is_leaf)
    {
        // 获取对应方向的位置
        position = (direction == SearchDirection::FORWARD) ?
                    current_node->forward_position : current_node->backward_position;
        return SearchResult::FOUND;
    }

    return SearchResult::PARTIAL;
}

uint8_t EncoderMap::get_bit_at_position(uint16_t position) const
{
    if(pattern_ == nullptr || position >= pattern_length_)
    {
        return 0; // 默认返回0
    }

    return pattern_[position];
}

size_t EncoderMap::get_memory_usage() const
{
    size_t total_memory = 0;

    // 计算对象本身的大小
    total_memory += sizeof(EncoderMap);

    // 计算搜索树的内存使用
    if(root_ != nullptr)
    {
        if(use_pool_)
        {
            // 内存池模式：计算实际使用的节点数
            total_memory += used_nodes_ * sizeof(TreeNode);
        }
        else
        {
            // 动态分配模式：递归计算节点数量
            total_memory += count_nodes(root_) * sizeof(TreeNode);
        }
    }

    return total_memory;
}

size_t EncoderMap::get_actual_node_count() const
{
    if(root_ == nullptr)
    {
        return 0;
    }

    if(use_pool_)
    {
        // 内存池模式：返回实际使用的节点数
        return used_nodes_;
    }
    else
    {
        // 动态分配模式：递归计算节点数量
        return count_nodes(root_);
    }
}

size_t EncoderMap::count_nodes(TreeNode *node) const
{
    if(node == nullptr)
    {
        return 0;
    }

    return 1 + count_nodes(node->left) + count_nodes(node->right);
}

TreeNode *EncoderMap::allocate_node()
{
    if(use_pool_)
    {
        // 内存池分配
        if(used_nodes_ >= pool_size_)
        {
            LOGW("Memory pool exhausted: used=%d, pool_size=%d", used_nodes_, pool_size_);
            return nullptr; // 内存池已满
        }
        LOGV("Allocated node from pool: index=%d", used_nodes_);
        return &node_pool_[used_nodes_++];
    }
    else
    {
        // 动态分配
        TreeNode *node = new TreeNode();
        LOGV("Allocated node dynamically: %p", node);
        return node;
    }
}

void EncoderMap::destroy_tree(TreeNode *node)
{
    if(node == nullptr || use_pool_)
    {
        return; // 内存池模式不需要释放
    }

    destroy_tree(node->left);
    destroy_tree(node->right);
    delete node;
}

void EncoderMap::print_tree_structure() const
{
    std::cout << "\n=== 搜索树结构信息 ===" << std::endl;
    std::cout << "图案长度: " << pattern_length_ << std::endl;
    std::cout << "最大搜索深度: " << (int)max_search_depth_ << std::endl;
    std::cout << "内存模式: " << (use_pool_ ? "内存池" : "动态分配") << std::endl;

    if(use_pool_)
    {
        std::cout << "已使用节点: " << used_nodes_ << "/" << pool_size_ << std::endl;
    }

    std::cout << "\n统一搜索树:" << std::endl;
    if(root_ != nullptr)
    {
        std::cout << "Root" << std::endl;
        if(root_->left)
        {
            print_tree_recursive(root_->left, "", false, 1);
        }
        if(root_->right)
        {
            print_tree_recursive(root_->right, "", true, 1);
        }
    }
    else
    {
        std::cout << "(空树)" << std::endl;
    }

    // 统计信息
    size_t total_nodes = get_actual_node_count();
    std::cout << "\n树统计信息:" << std::endl;
    std::cout << "总节点数: " << total_nodes << std::endl;
}

void EncoderMap::print_tree_recursive(TreeNode *node, const std::string &prefix, bool is_last, int depth) const
{
    if(node == nullptr || depth > 15)     // 增加深度限制
    {
        return;
    }

    // 打印当前节点
    std::cout << prefix;
    std::cout << (is_last ? "└── " : "├── ");
    std::cout << "Bit=" << (int)node->bit_value;

    if(node->is_forward_leaf || node->is_backward_leaf)
    {
        std::cout << " [";
        if(node->is_forward_leaf)
        {
            std::cout << "正向位置=" << node->forward_position;
        }
        if(node->is_forward_leaf && node->is_backward_leaf)
        {
            std::cout << ", ";
        }
        if(node->is_backward_leaf)
        {
            std::cout << "反向位置=" << node->backward_position;
        }
        std::cout << ", 深度=" << (int)node->depth << "]";
    }
    std::cout << std::endl;

    // 准备子节点的前缀
    std::string child_prefix = prefix + (is_last ? "    " : "│   ");

    // 统计子节点数量
    int child_count = 0;
    if(node->left)
    {
        child_count++;
    }
    if(node->right)
    {
        child_count++;
    }

    // 递归打印子节点
    int printed = 0;
    if(node->left)
    {
        printed++;
        print_tree_recursive(node->left, child_prefix, printed == child_count, depth + 1);
    }
    if(node->right)
    {
        printed++;
        print_tree_recursive(node->right, child_prefix, printed == child_count, depth + 1);
    }
}

void EncoderMap::build_forward_tree_iterative(const std::vector<uint16_t> &initial_starts)
{
    // 工作队列：存储(节点指针, 可能起始索引集合, 深度)
    struct WorkItem
    {
        TreeNode *node;
        std::vector<uint16_t> possible_starts;
        uint8_t depth;

        WorkItem(TreeNode *n, const std::vector<uint16_t> &starts, uint8_t d)
            : node(n), possible_starts(starts), depth(d) {}
    };

    std::queue<WorkItem> work_queue;
    work_queue.emplace(root_, initial_starts, 0);

    while(!work_queue.empty())
    {
        WorkItem current = work_queue.front();
        work_queue.pop();

        // 调试输出
        LOGV("Processing depth=%d, possible_starts_count=%zu", current.depth, current.possible_starts.size());

        // 如果集合大小为1，创建叶子节点
        if(current.possible_starts.size() == 1)
        {
            uint16_t start_index = current.possible_starts[0];
            current.node->is_forward_leaf = true;
            // 正向搜索：当前位置 = 起始位置 + 深度
            current.node->forward_position = (start_index + current.depth) % pattern_length_;
            LOGD("Created forward leaf: start=%d, depth=%d, current_pos=%d",
                    start_index, current.depth, current.node->forward_position);
            continue;
        }

        // 如果集合为空，跳过
        if(current.possible_starts.empty())
        {
            continue;
        }

        // 根据输入位0或1分割集合
        std::vector<uint16_t> starts_0, starts_1;

        for(uint16_t start_idx : current.possible_starts)
        {
            // 检查从起始位置start_idx开始，第depth个位置的值
            uint16_t check_pos = (start_idx + current.depth) % pattern_length_;
            if(pattern_[check_pos] == 0)
            {
                starts_0.push_back(start_idx);
            }
            else
            {
                starts_1.push_back(start_idx);
            }
        }

        // 只有当分割后的集合非空时才创建子节点
        if(!starts_0.empty())
        {
            if(current.node->left == nullptr)
            {
                current.node->left = allocate_node();
                if(current.node->left != nullptr)
                {
                    current.node->left->bit_value = 0;
                    current.node->left->parent = current.node;
                    current.node->left->depth = current.depth + 1;  // 设置深度信息
                }
            }
            if(current.node->left != nullptr)
            {
                work_queue.emplace(current.node->left, starts_0, current.depth + 1);
            }
        }

        if(!starts_1.empty())
        {
            if(current.node->right == nullptr)
            {
                current.node->right = allocate_node();
                if(current.node->right != nullptr)
                {
                    current.node->right->bit_value = 1;
                    current.node->right->parent = current.node;
                    current.node->right->depth = current.depth + 1;  // 设置深度信息
                }
            }
            if(current.node->right != nullptr)
            {
                work_queue.emplace(current.node->right, starts_1, current.depth + 1);
            }
        }
    }
}

void EncoderMap::build_backward_tree_iterative(const std::vector<uint16_t> &initial_starts)
{
    // 工作队列：存储(节点指针, 可能起始索引集合, 深度)
    struct WorkItem
    {
        TreeNode *node;
        std::vector<uint16_t> possible_starts;
        uint8_t depth;

        WorkItem(TreeNode *n, const std::vector<uint16_t> &starts, uint8_t d)
            : node(n), possible_starts(starts), depth(d) {}
    };

    std::queue<WorkItem> work_queue;
    work_queue.emplace(root_, initial_starts, 0);

    while(!work_queue.empty())
    {
        WorkItem current = work_queue.front();
        work_queue.pop();

        // 如果集合大小为1，创建叶子节点
        if(current.possible_starts.size() == 1)
        {
            uint16_t start_index = current.possible_starts[0];
            current.node->is_backward_leaf = true;
            // 反向搜索：当前位置 = 起始位置 - 深度
            uint16_t current_pos = (start_index - current.depth + pattern_length_) % pattern_length_;
            current.node->backward_position = current_pos;
            continue;
        }

        // 根据输入位0或1分割集合
        std::vector<uint16_t> starts_0, starts_1;

        for(uint16_t start_idx : current.possible_starts)
        {
            // 反向运动：从起始位置向前查看，检查位置为 start_idx - depth
            uint16_t check_pos = (start_idx - current.depth + pattern_length_) % pattern_length_;
            if(pattern_[check_pos] == 0)
            {
                starts_0.push_back(start_idx);
            }
            else
            {
                starts_1.push_back(start_idx);
            }
        }

        // 只有当分割后的集合非空时才创建子节点
        if(!starts_0.empty())
        {
            if(current.node->left == nullptr)
            {
                current.node->left = allocate_node();
                if(current.node->left != nullptr)
                {
                    current.node->left->bit_value = 0;
                    current.node->left->parent = current.node;
                    current.node->left->depth = current.depth + 1;  // 设置深度信息
                    work_queue.emplace(current.node->left, starts_0, current.depth + 1);
                }
            }
        }

        if(!starts_1.empty())
        {
            if(current.node->right == nullptr)
            {
                current.node->right = allocate_node();
                if(current.node->right != nullptr)
                {
                    current.node->right->bit_value = 1;
                    current.node->right->parent = current.node;
                    current.node->right->depth = current.depth + 1;  // 设置深度信息
                    work_queue.emplace(current.node->right, starts_1, current.depth + 1);
                }
            }
        }
    }
}

} // namespace AbsoluteEncoder