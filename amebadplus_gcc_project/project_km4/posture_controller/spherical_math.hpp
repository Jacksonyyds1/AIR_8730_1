#ifndef __SPHERICAL_MATH_HPP
#define __SPHERICAL_MATH_HPP

#include <cmath>
#include <algorithm>

#include "arm_math.h"
#include "quaternion_functions.h"

#include "rad_deg.hpp"
#include "spherical_math.hpp"
#include "vector_math.hpp"

/**
 * @file spherical_math.hpp
 * @brief 球坐标系数学运算库内部C++接口
 *
 * 提供C++风格的球坐标运算功能，供其他模块内部使用
 */

namespace GraphicsMath {

/**
 * @brief 球坐标结构
 */
class SphericalCoord {
public:
__attribute__((aligned(4))) float32_t azimuth, elevation; // 方位角、仰角(角度制)

SphericalCoord(float32_t azimuth = 0.0f, float32_t elevation = 0.0f)
    : azimuth(azimuth), elevation(elevation) {}

SphericalCoord(float32_t* arr)
    : azimuth(arr[0]), elevation(arr[1]) {}

SphericalCoord(const SphericalCoord& other)
    : azimuth(other.azimuth), elevation(other.elevation) {}

SphericalCoord(const Vector3& vec) {
    float32_t elevation_rad, azimuth_rad;
    Vector3 normalized = vec.normalized();

    if(normalized == Vector3(0.0f, 0.0f, 0.0f)) {
        azimuth = 0.0f;
        elevation = 0.0f;
        return;
    }
    
    // 计算仰角、方位角
    elevation_rad = std::asin(normalized.z);
    arm_atan2_f32(normalized.y, normalized.x, &azimuth_rad);

    azimuth = rad2deg(azimuth_rad);
    elevation = rad2deg(elevation_rad);
}

/**
 * @brief 从角度坐标系转换为空间向量坐标系
 */
Vector3 toVector(void) const {
    float32_t ce, se;
    float32_t ca, sa;
    
    arm_sin_cos_f32(elevation, &se, &ce);
    arm_sin_cos_f32(azimuth, &sa, &ca);
    
    return Vector3(
        ce * ca,
        ce * sa,
        se
    );
}

// 归一化方位角
float32_t normalizeAzimuth(void) const {
    float32_t norm_azimuth = std::fmod(azimuth, 360.0f);
    if (norm_azimuth < 0.0f) {
        norm_azimuth += 360.0f;
    }
    return norm_azimuth;
}

// 就地归一化方位角
float32_t& normalizeAzimuth(void) {
    azimuth = std::fmod(azimuth, 360.0f);
    if (azimuth < 0.0f) {
        azimuth += 360.0f;
    }
    return azimuth;
}
};

} // namespace GraphicsMath

#endif /* __SPHERICAL_MATH_HPP */
