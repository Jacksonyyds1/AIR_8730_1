#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
德布鲁因序列生成器
支持k值到10，通过命令行交互输入参数
生成德布鲁因序列作为码盘图案
"""

import sys
import os
from collections import defaultdict


def generate_debruijn_sequence(k):
    """
    使用Martin算法生成德布鲁因序列 B(k, 2)
    
    Args:
        k: 子序列长度
    
    Returns:
        德布鲁因序列字符串
    """
    if k == 1:
        return "01"
    
    # 构建de Bruijn图
    # 节点是长度为k-1的字符串
    # 边表示长度为k的字符串
    
    # 生成所有长度为k-1的节点
    nodes = []
    for i in range(2**(k-1)):
        node = bin(i)[2:].zfill(k-1)
        nodes.append(node)
    
    # 构建邻接表
    graph = defaultdict(list)
    in_degree = defaultdict(int)
    out_degree = defaultdict(int)
    
    for node in nodes:
        # 对每个节点，添加两条出边（添加0或1）
        for bit in ['0', '1']:
            next_node = node[1:] + bit
            graph[node].append((next_node, bit))
            out_degree[node] += 1
            in_degree[next_node] += 1
    
    # 使用Hierholzer算法找欧拉回路
    def find_eulerian_path():
        # 深拷贝图，因为我们会修改它
        temp_graph = defaultdict(list)
        for node in graph:
            temp_graph[node] = graph[node][:]
        
        # 选择起始节点（全0节点）
        start_node = '0' * (k-1)
        
        # Hierholzer算法
        stack = [start_node]
        path = []
        
        while stack:
            curr = stack[-1]
            if temp_graph[curr]:
                next_node, edge_label = temp_graph[curr].pop()
                stack.append(next_node)
            else:
                path.append(stack.pop())
        
        path.reverse()
        return path
    
    # 获取欧拉路径
    euler_path = find_eulerian_path()
      # 从欧拉路径构建德布鲁因序列
    if len(euler_path) < 2:
        return '0' * (2**k)
    
    # 重建图来获取边标签
    edge_labels = {}
    for node in nodes:
        for next_node, bit in graph[node]:
            edge_labels[(node, next_node)] = bit
    
    # 构建序列 - 只取边的标签，不包含起始节点
    debruijn = ""
    
    for i in range(1, len(euler_path)):
        prev_node = euler_path[i-1]
        curr_node = euler_path[i]
        
        if (prev_node, curr_node) in edge_labels:
            debruijn += edge_labels[(prev_node, curr_node)]
    
    # 德布鲁因序列应该恰好是 2^k 位
    # 如果长度不对，截取或补充
    expected_length = 2**k
    if len(debruijn) > expected_length:
        debruijn = debruijn[:expected_length]
    elif len(debruijn) < expected_length:
        # 这种情况不应该发生在正确的算法中
        debruijn = debruijn + '0' * (expected_length - len(debruijn))
    
    return debruijn


def save_to_file(sequence, filename):
    """
    将序列保存到文件
    
    Args:
        sequence: 要保存的序列
        filename: 文件名
    
    Returns:
        bool: 保存是否成功
    """
    try:
        with open(filename, 'w', encoding='utf-8') as f:
            f.write(sequence)
        print(f"✓ 序列已保存到文件: {filename}")
        print(f"  文件路径: {os.path.abspath(filename)}")
        return True
    except Exception as e:
        print(f"✗ 保存文件失败: {e}")
        return False


def verify_debruijn_sequence(sequence, k):
    """
    验证德布鲁因序列的正确性
    
    Args:
        sequence: 德布鲁因序列
        k: k值
    
    Returns:
        tuple: (是否正确, 实际k-mer数量, 预期k-mer数量)
    """
    unique_kmers = set()
    for i in range(len(sequence)):
        kmer = sequence[i:i+k]
        if len(kmer) == k:
            unique_kmers.add(kmer)
    
    expected_count = 2**k
    actual_count = len(unique_kmers)
    
    return actual_count == expected_count, actual_count, expected_count


def get_yes_no_input(prompt, default=True):
    """获取是/否输入"""
    default_str = "Y/n" if default else "y/N"
    while True:
        response = input(f"{prompt} ({default_str}): ").strip().lower()
        if not response:
            return default
        if response in ['y', 'yes', '是', '1']:
            return True
        elif response in ['n', 'no', '否', '0']:
            return False
        else:
            print("请输入 y/yes/是 或 n/no/否")


def main():
    """主函数 - 交互式模式"""
    print("=" * 60)
    print("           德布鲁因序列生成器")
    print("=" * 60)
    print("此工具将生成德布鲁因序列")
    print("德布鲁因序列：每个长度为k的二进制子序列在序列中恰好出现一次")
    print("支持k值范围：1-10")
    print()
    
    # 获取k值
    while True:
        try:
            k_input = input("请输入k值 (1-10): ")
            k = int(k_input)
            if k <= 0:
                print("❌ 错误: k值必须为正整数")
                continue
            elif k > 10:
                print("❌ 错误: k值不能超过10（序列会过长）")
                continue
            break
        except ValueError:
            print("❌ 错误: 请输入有效的整数")
        except KeyboardInterrupt:
            print("\n\n👋 程序已退出")
            sys.exit(0)
    
    print(f"\n📊 生成参数: k = {k}")
    print(f"   德布鲁因序列长度: 2^{k} = {2**k} 位")
    
    if k >= 8:
        print(f"⚠️  警告: k={k} 会生成很长的序列 ({2**k} 位)")
        if not get_yes_no_input("确定要继续吗？", False):
            print("👋 程序已退出")
            return
    
    # 生成序列
    print(f"\n⚙️  正在生成德布鲁因序列...")
    
    try:
        # 生成德布鲁因序列
        debruijn_seq = generate_debruijn_sequence(k)
        
        print(f"✓ 德布鲁因序列生成成功")
        print(f"   长度: {len(debruijn_seq)} 位")
        
        # 验证序列
        is_valid, actual_count, expected_count = verify_debruijn_sequence(debruijn_seq, k)
        if is_valid:
            print(f"✓ 序列验证通过 ({actual_count}/{expected_count} k-mers)")
        else:
            print(f"⚠️  序列验证警告: {actual_count}/{expected_count} k-mers")
        
        final_sequence = debruijn_seq
        
        # 显示序列
        print(f"\n📋 生成的序列:")
        print("=" * 60)
        
        if len(final_sequence) <= 200:
            print(final_sequence)
        else:
            print(f"{final_sequence[:100]}...{final_sequence[-100:]}")
            print(f"(序列太长，仅显示前100位和后100位)")
        
        print("=" * 60)
          # 保存到文件
        default_filename = f"debruijn_k{k}.txt"
        
        print(f"\n💾 文件保存")
        filename_input = input(f"   输出文件名 (默认: {default_filename}): ").strip()
        filename = filename_input if filename_input else default_filename
        
        save_to_file(final_sequence, filename)
        
        print(f"\n🎉 生成完成！")
        print(f"   序列类型: 德布鲁因序列")
        print(f"   序列长度: {len(final_sequence)} 位")
        
    except Exception as e:
        print(f"❌ 生成序列时发生错误: {e}")
        import traceback
        traceback.print_exc()
        return


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n\n👋 程序已被用户中断")
        sys.exit(0)