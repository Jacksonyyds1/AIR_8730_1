#ifndef __QUATERNION_MATH_HPP
#define __QUATERNION_MATH_HPP

#include <cmath>
#include <algorithm>

#include "arm_math.h"

#include "rad_deg.hpp"
#include "vector_math.hpp"
#include "quaternion_math.hpp"

/**
 * @file quaternion_math.hpp
 * @brief 四元数数学运算库内部C++接口
 *
 * 提供C++风格的四元数运算类和操作符重载，供其他模块内部使用
 */

namespace GraphicsMath {

/**
 * @brief C++风格的四元数类
 */
class Quaternion {
public:
__attribute__((aligned(4))) float32_t w, x, y, z;
operator float32_t*() { return &w; }

// 构造函数
Quaternion(float32_t w = 1.0f, float32_t x = 0.0f, float32_t y = 0.0f, float32_t z = 0.0f) 
    : w(w), x(x), y(y), z(z) {}
Quaternion(float32_t *arr) 
    : w(arr[0]), x(arr[1]), y(arr[2]), z(arr[3]) {}
Quaternion(const Quaternion& other) 
    : w(other.w), x(other.x), y(other.y), z(other.z) {}

// 基础运算操作符
Quaternion operator+(const Quaternion& other) const {
    Quaternion result;
    arm_add_f32((float32_t *)this, (float32_t *)&other, (float32_t *)&result, 4);
    return result;
}

Quaternion operator-(const Quaternion& other) const {
    Quaternion result;
    arm_sub_f32((float32_t *)this, (float32_t *)&other, (float32_t *)&result, 4);
    return result;
}

Quaternion operator*(const Quaternion& other) const {
    Quaternion result;
    // 注意CMSIS的参数顺序是 (p, q, p*q)
    arm_quaternion_product_single_f32((float32_t *)this, (float32_t *)&other, (float32_t *)&result);
    return result;
}

Quaternion operator*(float32_t scalar) const {
    Quaternion result;
    arm_scale_f32((float32_t *)this, scalar, (float32_t *)&result, 4);
    return result;
}

Quaternion operator/(float32_t scalar) const {
    Quaternion result;
    arm_scale_f32((float32_t *)this, 1.0f / scalar, (float32_t *)&result, 4);
    return result;
}

bool operator==(const Quaternion& other) const {
    return std::abs(w - other.w) < 1e-6f &&
           std::abs(x - other.x) < 1e-6f &&
           std::abs(y - other.y) < 1e-6f &&
           std::abs(z - other.z) < 1e-6f;
}

// 点积
float32_t dot(const Quaternion& other) const {
    float32_t result;
    arm_dot_prod_f32((float32_t *)this, (float32_t *)&other, 4, &result);
    return result;
}

// 模长
float32_t magnitude() const {
    float32_t mag_sq, mag;
    arm_power_f32((float32_t *)this, 4, &mag_sq);
    arm_sqrt_f32(mag_sq, &mag);
    return mag;
}

// 归一化
Quaternion normalized() const {
    Quaternion result;
    arm_quaternion_normalize_f32((float32_t *)this, (float32_t *)&result, 1);
    return result;
}

// 就地归一化
Quaternion& normalized() {
    arm_quaternion_normalize_f32((float32_t *)this, (float32_t *)this, 1);
    return *this;
}

// 共轭
Quaternion conjugate() const {
    return Quaternion(w, -x, -y, -z);
}

// 逆
Quaternion inverse() const {
    Quaternion result;
    arm_quaternion_inverse_f32((float32_t *)this, (float32_t *)&result, 1);
    return result;
}

// 旋转向量
Vector3 rotateVector(const Vector3& v) const {
    Quaternion qv(0, v.x, v.y, v.z); // 将向量转换为四元数
    Quaternion qConjugate = conjugate();
    Quaternion result = (*this * qv) * qConjugate; // 使用四元数旋转公式
    return Vector3(result.x, result.y, result.z); // 返回旋转后的向量
}

// 从旋转轴和角度创建
static Quaternion fromAxisAngle(const Vector3& axis, float32_t angle_deg) {
    Vector3 normalizedAxis = axis.normalized();
    float32_t halfAngle = angle_deg * 0.5f;
    float32_t sinHalf, cosHalf;
    arm_sin_cos_f32(halfAngle, &sinHalf, &cosHalf);
    
    return Quaternion(cosHalf, 
                        normalizedAxis.x * sinHalf,
                        normalizedAxis.y * sinHalf,
                        normalizedAxis.z * sinHalf);
}

// 从两个向量之间的旋转创建
static Quaternion fromVectors(const Vector3& from, const Vector3& to) {
    Vector3 fromNorm = from.normalized();
    Vector3 toNorm = to.normalized();
    
    float32_t dotProduct = fromNorm.dot(toNorm);
    
    // 处理平行情况
    if (dotProduct >= 0.999999f) {
        return Quaternion(1, 0, 0, 0); // 无旋转
    }
    
    // 处理反向情况
    if (dotProduct <= -0.999999f) {
        // 找一个垂直轴
        Vector3 axis = Vector3(1, 0, 0) * fromNorm;
        if (axis.magnitude() < 1e-6f) {
            axis = Vector3(0, 1, 0) * fromNorm;
        }
        axis.normalized();
        return fromAxisAngle(axis, PI);
    }
    
    Vector3 crossProduct = fromNorm * toNorm;
    float32_t w = 1.0f + dotProduct;
    
    Quaternion result(w, crossProduct.x, crossProduct.y, crossProduct.z);
    return result.normalized();
}

// 球面线性插值
static Quaternion slerp(const Quaternion& q1, const Quaternion& q2, float32_t t) {
    Quaternion quat2 = q2;
    
    // 计算点积
    float32_t dotProduct = q1.dot(q2);
    
    // 如果点积为负，取反以确保最短路径
    if (dotProduct < 0.0f) {
        quat2 = q2 * -1.0f;
        dotProduct = -dotProduct;
    }
    
    // 如果角度很小，使用线性插值
    if (dotProduct > 0.9995f) {
        Quaternion result = q1 + (quat2 - q1) * t;
        return result.normalized();
    }
    
    // 球面线性插值
    float32_t sinTheta_sq = 1.0f - dotProduct * dotProduct;
    float32_t sinTheta;
    arm_sqrt_f32(sinTheta_sq, &sinTheta);
    
    // 计算插值因子
    float32_t theta = std::asin(sinTheta);

    float32_t sin_t1_theta, sin_t2_theta;
    sin_t1_theta = arm_sin_f32((1.0f - t) * theta);
    sin_t2_theta = arm_sin_f32(t * theta);

    float32_t factor1 = sin_t1_theta / sinTheta;
    float32_t factor2 = sin_t2_theta / sinTheta;
    
    return q1 * factor1 + quat2 * factor2;
}

// 高性能 SLERP - 避免反三角函数
static Quaternion slerpFast(const Quaternion& q1, const Quaternion& q2, float32_t t) {
    Quaternion quat2 = q2;
    
    float32_t dotProduct;
    arm_dot_prod_f32((float32_t*)&q1, (float32_t*)&q2, 4, &dotProduct);
    
    if (dotProduct < 0.0f) {
        quat2 = q2 * -1.0f;
        dotProduct = -dotProduct;
    }
    
    // 线性插值阈值
    if (dotProduct > 0.9995f) {
        Quaternion result = q1 + (quat2 - q1) * t;
        return result.normalized();
    }
    
    // 使用有理逼近避免三角函数
    float32_t theta_approx = 1.0f - dotProduct;  // 近似角度
    float32_t s = t * theta_approx / (theta_approx + (1.0f - t) * theta_approx);
    
    // 简化的权重计算
    float32_t factor1 = 1.0f - s;
    float32_t factor2 = s;
    
    Quaternion result = q1 * factor1 + quat2 * factor2;
    return result.normalized();
}

// 从欧拉角创建（ZYX顺序）
static Quaternion fromEuler(float32_t roll, float32_t pitch, float32_t yaw) {
    float32_t cr, sr;
    float32_t cp, sp;
    float32_t cy, sy;

    arm_sin_cos_f32(roll * 0.5f, &sr, &cr);
    arm_sin_cos_f32(pitch * 0.5f, &sp, &cp);
    arm_sin_cos_f32(yaw * 0.5f, &sy, &cy);
    
    return Quaternion(
        cr * cp * cy + sr * sp * sy,
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy
    );
}

// 转换为欧拉角
Vector3 toEuler() const {
    Vector3 euler;
    
    // Roll (x-axis rotation)
    float32_t sinr_cosp = 2 * (w * x + y * z);
    float32_t cosr_cosp = 1 - 2 * (x * x + y * y);
    arm_atan2_f32(sinr_cosp, cosr_cosp, &euler.x);
    
    // Pitch (y-axis rotation)
    float32_t sinp = 2 * (w * y - z * x);
    if (std::abs(sinp) >= 1)
        euler.y = std::copysign(PI / 2, sinp);
    else
        euler.y = std::asin(sinp);
    
    // Yaw (z-axis rotation)
    float32_t siny_cosp = 2 * (w * z + x * y);
    float32_t cosy_cosp = 1 - 2 * (y * y + z * z);
    arm_atan2_f32(siny_cosp, cosy_cosp, &euler.z);
    
    return euler;
}

// 检查是否归一化
bool isNormalized(float32_t tolerance = 1e-6f) const {
    float32_t norm = magnitude();
    return std::abs(norm - 1.0f) < tolerance;
}

// 检查相等性
bool equals(const Quaternion& other, float32_t tolerance = 1e-6f) const {
    return std::abs(w - other.w) < tolerance &&
            std::abs(x - other.x) < tolerance &&
            std::abs(y - other.y) < tolerance &&
            std::abs(z - other.z) < tolerance;
}

};

// 全局操作符
inline Quaternion operator*(float32_t scalar, const Quaternion& q) {
    return q * scalar;
}

} // namespace GraphicsMath

#endif /* __QUATERNION_MATH_HPP */
