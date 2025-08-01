#ifndef ENCODER_MAP_HPP
#define ENCODER_MAP_HPP

#include <cstdint>
#include <memory>
#include <vector>
#include <string>

namespace AbsoluteEncoder
{

// 搜索结果状态
enum class SearchResult
{
    FOUND,          // 找到完整匹配
    PARTIAL,        // 部分匹配，可继续搜索
    NOT_FOUND       // 未找到匹配
};

// 搜索方向
enum class SearchDirection
{
    FORWARD,        // 正向搜索
    BACKWARD        // 反向搜索
};

// 二叉树节点结构
struct TreeNode
{
    uint16_t forward_position;      // 正向搜索完成时编码器的当前位置
    uint16_t backward_position;     // 反向搜索完成时编码器的当前位置
    TreeNode *left;                 // 0分支
    TreeNode *right;                // 1分支
    TreeNode *parent;               // 父节点
    bool is_forward_leaf;           // 是否为正向搜索的叶子节点
    bool is_backward_leaf;          // 是否为反向搜索的叶子节点
    uint8_t bit_value;              // 当前位的值 (0 或 1)
    uint8_t depth;                  // 节点深度（从根节点开始计数）

    TreeNode() : forward_position(0), backward_position(0),
        left(nullptr), right(nullptr), parent(nullptr),
        is_forward_leaf(false), is_backward_leaf(false), bit_value(0), depth(0) {}
};

class EncoderMap
{
public:
    /**
     * 构造函数（动态内存版本）
     * @param pattern 编码器图案数组
     * @param pattern_length 图案长度
     * @param max_search_depth 最大搜索深度限制
     */
    EncoderMap(const uint8_t *pattern, uint16_t pattern_length, uint8_t max_search_depth);

    /**
     * 构造函数（内存池版本，RTOS适用）
     * @param pattern 编码器图案数组
     * @param pattern_length 图案长度
     * @param max_search_depth 最大搜索深度限制
     * @param node_pool 预分配的节点内存池
     * @param pool_size 内存池大小（节点数量）
     */
    EncoderMap(const uint8_t *pattern, uint16_t pattern_length, uint8_t max_search_depth,
                TreeNode *node_pool, uint16_t pool_size);

    /**
     * 析构函数
     */
    ~EncoderMap();

    /**
     * 开始新的搜索
     * @param bit 输入的位值
     * @param direction 搜索方向
     * @return 搜索上下文节点
     */
    TreeNode *start_search(uint8_t bit, SearchDirection direction);

    /**
     * 绝对搜索
     * @param current_node 当前搜索上下文
     * @param bit 新输入的位值
     * @param direction 搜索方向
     * @param position 输出参数：如果找到匹配，返回位置
     * @return 搜索结果状态
     */
    SearchResult absolute_search(TreeNode *&current_node, uint8_t bit,
                                    SearchDirection direction, uint16_t &position);

    /**
     * 获取指定位置的位值
     * @param position 位置索引
     * @return 位值 (0 或 1)
     */
    uint8_t get_bit_at_position(uint16_t position) const;

    /**
     * 获取图案长度
     */
    uint16_t get_pattern_length() const
    {
        return pattern_length_;
    }

    /**
     * 获取最大搜索深度
     */
    uint8_t get_max_search_depth() const
    {
        return max_search_depth_;
    }

    /**
     * 获取实际使用的内存大小
     */
    size_t get_memory_usage() const;

    /**
     * 获取实际节点数量
     */
    size_t get_actual_node_count() const;

    /**
     * 输出搜索树结构信息
     */
    void print_tree_structure() const;

    /**
     * 获取内存池使用情况（仅内存池模式有效）
     */
    uint16_t get_used_nodes() const
    {
        return used_nodes_;
    }
    uint16_t get_pool_size() const
    {
        return pool_size_;
    }
    bool is_pool_mode() const
    {
        return use_pool_;
    }
    bool is_pool_full() const
    {
        return use_pool_ && used_nodes_ >= pool_size_;
    }

    /**
     * 获取图案数据指针
     */
    const uint8_t *get_pattern_data() const
    {
        return pattern_;
    }

private:
    const uint8_t *pattern_;            // 编码器图案
    uint16_t pattern_length_;           // 图案长度
    uint8_t max_search_depth_;          // 最大搜索深度限制
    TreeNode *root_;                    // 统一搜索树根节点

    // 内存池相关
    bool use_pool_;                     // 是否使用内存池模式
    TreeNode *node_pool_;               // 内存池指针
    uint16_t pool_size_;                // 内存池大小
    uint16_t used_nodes_;               // 已使用节点数量

    /**
     * 构建搜索树
     */
    void build_search_tree();

    /**
     * 构建正向序列的搜索路径
     */
    void build_forward_sequences();

    /**
     * 构建反向序列的搜索路径
     */
    void build_backward_sequences();

    /**
     * 非递归构建正向运动的搜索树（向右运动）
     * @param initial_starts 初始可能的起始索引集合
     */
    void build_forward_tree_iterative(const std::vector<uint16_t> &initial_starts);

    /**
     * 非递归构建反向运动的搜索树（向左运动，在原序列上）
     * @param initial_starts 初始可能的起始索引集合
     */
    void build_backward_tree_iterative(const std::vector<uint16_t> &initial_starts);

    /**
     * 检查子图是否在码盘上唯一
     * @param start_pos 起始位置
     * @param length 子图长度
     * @param direction 搜索方向
     * @return 是否唯一
     */
    bool is_subsequence_unique(uint16_t start_pos, uint8_t length, SearchDirection direction) const;

    /**
     * 检查是否存在更短的唯一子图，且该短子图完全包含在当前子图序列中
     * @param start_pos 起始位置
     * @param length 子图长度
     * @param direction 搜索方向
     * @return 是否存在包含的更短唯一子图
     */
    bool has_shorter_unique_subsequence(uint16_t start_pos, uint8_t length, SearchDirection direction) const;

    /**
     * 将唯一子图插入搜索树
     * @param start_pos 起始位置
     * @param length 子图长度
     * @param direction 搜索方向
     */
    void insert_unique_subsequence(uint16_t start_pos, uint8_t length, SearchDirection direction);

    /**
     * 递归释放树节点
     */
    void destroy_tree(TreeNode *node);

    /**
     * 统计树中节点数量
     * @param node 根节点
     * @return 节点数量
     */
    size_t count_nodes(TreeNode *node) const;

    /**
     * 分配节点（根据模式选择动态分配或内存池分配）
     */
    TreeNode *allocate_node();

    /**
     * 递归打印树结构
     * @param node 当前节点
     * @param prefix 前缀字符串
     * @param is_last 是否为最后一个子节点
     * @param depth 当前深度
     */
    void print_tree_recursive(TreeNode *node, const std::string &prefix, bool is_last, int depth) const;

    // 禁用拷贝构造和赋值
    EncoderMap(const EncoderMap &) = delete;
    EncoderMap &operator=(const EncoderMap &) = delete;
};

} // namespace AbsoluteEncoder

#endif // ENCODER_MAP_HPP