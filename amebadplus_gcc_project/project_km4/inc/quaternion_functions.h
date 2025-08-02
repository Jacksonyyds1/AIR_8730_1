/**
 * @file error_fixes.h
 * @brief 修复所有编译错误的解决方案
 */

// ===============================================
// 1. 修复 quaternion_functions.h 中的函数冲突
// ===============================================

#ifndef QUATERNION_FUNCTIONS_FIXED_H
#define QUATERNION_FUNCTIONS_FIXED_H

#include "arm_math.h"

#ifdef __cplusplus
extern "C" {
#endif

// 四元数乘法 - 保持不变
static inline void arm_quaternion_product_single_f32(const float32_t *qa, 
                                                     const float32_t *qb, 
                                                     float32_t *qr)
{
    qr[0] = qa[0] * qb[0] - qa[1] * qb[1] - qa[2] * qb[2] - qa[3] * qb[3];
    qr[1] = qa[0] * qb[1] + qa[1] * qb[0] + qa[2] * qb[3] - qa[3] * qb[2];
    qr[2] = qa[0] * qb[2] - qa[1] * qb[3] + qa[2] * qb[0] + qa[3] * qb[1];
    qr[3] = qa[0] * qb[3] + qa[1] * qb[2] - qa[2] * qb[1] + qa[3] * qb[0];
}

// 四元数归一化 - 保持不变
static inline void arm_quaternion_normalize_f32(const float32_t *pInputQuaternions,
                                                float32_t *pNormalizedQuaternions,
                                                uint32_t nbQuaternions)
{
    uint32_t i;
    for (i = 0; i < nbQuaternions; i++) {
        const float32_t *input = &pInputQuaternions[i * 4];
        float32_t *output = &pNormalizedQuaternions[i * 4];
        
        float32_t power_sum, norm;
        arm_power_f32(input, 4, &power_sum);
        
        if (power_sum < 1e-8f) {
            output[0] = 1.0f;
            output[1] = output[2] = output[3] = 0.0f;
        } else {
            arm_sqrt_f32(power_sum, &norm);
            arm_scale_f32(input, 1.0f / norm, output, 4);
        }
    }
}

// 四元数求逆 - 保持不变
static inline void arm_quaternion_inverse_f32(const float32_t *pInputQuaternions,
                                              float32_t *pInverseQuaternions,
                                              uint32_t nbQuaternions)
{
    uint32_t i;
    for (i = 0; i < nbQuaternions; i++) {
        const float32_t *input = &pInputQuaternions[i * 4];
        float32_t *output = &pInverseQuaternions[i * 4];
        
        float32_t norm_squared;
        arm_power_f32(input, 4, &norm_squared);
        
        if (norm_squared < 1e-8f) {
            output[0] = 1.0f;
            output[1] = output[2] = output[3] = 0.0f;
        } else {
            output[0] = input[0] / norm_squared;
            output[1] = -input[1] / norm_squared;
            output[2] = -input[2] / norm_squared;
            output[3] = -input[3] / norm_squared;
        }
    }
}

// atan2 函数 - 保持不变
static inline void arm_atan2_f32(float32_t y, float32_t x, float32_t *result)
{
    *result = atan2f(y, x);
}

// 删除 arm_sin_cos_f32 函数定义，因为 RTL8721DCM 已经有了！
// 注释掉或删除这个函数，因为它与系统库冲突
/*
static inline void arm_sin_cos_f32(float32_t theta, float32_t *pSinVal, float32_t *pCosVal)
{
    *pSinVal = arm_sin_f32(theta);
    *pCosVal = arm_cos_f32(theta);
}
*/

#ifdef __cplusplus
}
#endif

#endif /* QUATERNION_FUNCTIONS_FIXED_H */