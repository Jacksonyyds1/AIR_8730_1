#ifndef __VECTOR_MATH_HPP
#define __VECTOR_MATH_HPP

#include <cmath>
#include <algorithm>

#include "arm_math.h"

#include "rad_deg.hpp"
#include "vector_math.hpp"

/**
 * @file vector_math.hpp
 * @brief 向量数学运算库内部C++接口
 *
 * 提供C++风格的向量运算类和操作符重载，供其他模块内部使用
 */

namespace GraphicsMath {

/**
 * @brief C++风格的三维向量类
 */
class Vector3 {
public:
__attribute__((aligned(4))) float32_t x, y, z;
operator float32_t *() { return &x; } // 允许直接转换为float指针

// 构造函数
Vector3(float32_t x = 0.0f, float32_t y = 0.0f, float32_t z = 0.0f) : x(x), y(y), z(z) {}
Vector3(float32_t* arr) : x(arr[0]), y(arr[1]), z(arr[2]) {}
Vector3(const Vector3& other) : x(other.x), y(other.y), z(other.z) {}

// 基础运算操作符
Vector3 operator+(const Vector3& other) const {
    Vector3 result;
    arm_add_f32((float32_t *)this, (float32_t *)&other, (float32_t *)&result, 3);
    return result;
}

Vector3 operator-(const Vector3& other) const {
    Vector3 result;
    arm_sub_f32((float32_t *)this, (float32_t *)&other, (float32_t *)&result, 3);
    return result;
}

Vector3 operator*(const Vector3& other) const {
    return Vector3(
        y * other.z - z * other.y,
        z * other.x - x * other.z,
        x * other.y - y * other.x
    );
}

Vector3 operator*(float32_t scalar) const {
    Vector3 result;
    arm_scale_f32((float32_t *)this, scalar, (float32_t *)&result, 3);
    return result;
}

Vector3 operator/(float32_t scalar) const {
    Vector3 result;
    arm_scale_f32((float32_t *)this, 1.0f / scalar, (float32_t *)&result, 3);
    return result;
}

bool operator==(const Vector3& other) const {
    return std::abs(x - other.x) < 1e-6f &&
           std::abs(y - other.y) < 1e-6f &&
           std::abs(z - other.z) < 1e-6f;
}

Vector3& operator+=(const Vector3& other) {
    arm_add_f32((float32_t *)this, (float32_t *)&other, (float32_t *)this, 3);
    return *this;
}

Vector3& operator-=(const Vector3& other) {
    arm_sub_f32((float32_t *)this, (float32_t *)&other, (float32_t *)this, 3);
    return *this;
}

Vector3& operator*=(const Vector3& other) {
    float32_t x_temp = y * other.z - z * other.y;
    float32_t y_temp = z * other.x - x * other.z;
    float32_t z_temp = x * other.y - y * other.x;
    x = x_temp;
    y = y_temp;
    z = z_temp;
    return *this;
}

Vector3& operator*=(float32_t scalar) {
    arm_scale_f32((float32_t *)this, scalar, (float32_t *)this, 3);
    return *this;
}

Vector3 operator/=(float32_t scalar) {
    arm_scale_f32((float32_t *)this, 1.0f / scalar, (float32_t *)this, 3);
    return *this;
}

// 点积
float32_t dot(const Vector3& other) const {
    float32_t result;
    arm_dot_prod_f32((float32_t *)this, (float32_t *)&other, 3, &result);
    return result;
}

// 模长
float32_t magnitude() const {
    float32_t mag_sq, mag;
    arm_power_f32((float32_t *)this, 3, &mag_sq);
    arm_sqrt_f32(mag_sq, &mag);
    return mag;
}

// 归一化
Vector3 normalized() const {
    float32_t mag = magnitude();
    if (mag < 1e-8f) return Vector3(0, 0, 0);
    Vector3 result;
    arm_scale_f32((float32_t*)this, 1.0f / mag, (float32_t*)&result, 3);
    return result;
}

// 就地归一化
Vector3& normalized() {
    float32_t mag = magnitude();
    if (mag >= 1e-8f) {
        arm_scale_f32((float32_t*)this, 1.0f / mag, (float32_t*)this, 3);
    } else {
        x = y = z = 0.0f;
    }
    return *this;
}

// 共轭
Vector3 conjugate() const {
    return Vector3(x, -y, -z);
}

// 逆
Vector3 inverse() const {
    return Vector3(-x, -y, -z);
}

// 线性插值
static Vector3 lerp(const Vector3& a, const Vector3& b, float32_t t) {
    return a + (b - a) * t;
}

// 实例方法的线性插值
Vector3 lerp(const Vector3& other, float32_t t) const {
    return lerp(*this, other, t);
}

// 计算与另一个向量的夹角（弧度）
float32_t angleTo(const Vector3& other) const {
    Vector3 a = normalized();
    Vector3 b = other.normalized();
    float32_t dotProduct = a.dot(b);
    // 限制范围防止数值误差
    dotProduct = std::max(-1.0f, std::min(1.0f, dotProduct));
    return std::acos(dotProduct);
}

// 检查两个向量是否相等（在容差范围内）
bool equals(const Vector3& other, float32_t tolerance = 1e-6f) const {
    return std::abs(x - other.x) < tolerance &&
            std::abs(y - other.y) < tolerance &&
            std::abs(z - other.z) < tolerance;
}

// 检查是否为零向量
bool isZero(float32_t epsilon = 1e-8f) const {
    return magnitude() < epsilon;
}

// 检查是否为单位向量
bool isUnit(float32_t epsilon = 1e-6f) const {
    return std::abs(magnitude() - 1.0f) < epsilon;
}

// 向量距离计算
float32_t distanceTo(const Vector3& other) const {
    Vector3 diff = *this - other;
    return diff.magnitude();
}

// DSP优化的向量投影
Vector3 projectOnto(const Vector3& other) const {
    Vector3 norm_other = other.normalized();
    float32_t projection = dot(norm_other);
    return norm_other * projection;
}

// 向量拒绝（投影的垂直分量）
Vector3 rejectFrom(const Vector3& other) const {
    return *this - projectOnto(other);
}
};

// 全局操作符
inline Vector3 operator*(float32_t scalar, const Vector3& v) {
    return v * scalar;
}

} // namespace GraphicsMath

#endif /* __VECTOR_MATH_HPP */
