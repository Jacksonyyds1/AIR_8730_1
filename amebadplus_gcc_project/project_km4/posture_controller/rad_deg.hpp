#ifndef __RAD_DEG_HPP
#define __RAD_DEG_HPP

#include <cmath>
#include "arm_math.h"

// 常量定义
constexpr float32_t deg2rad(float32_t deg) { return (deg * PI / 180.0f); }
constexpr float32_t rad2deg(float32_t rad) { return (rad * 180.0f / PI); }

#endif /* __RAD_DEG_HPP */