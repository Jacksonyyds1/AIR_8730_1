#ifndef __POSTURE_H
#define __POSTURE_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#include "vector_math.hpp"
#include "quaternion_math.hpp"
#include "spherical_math.hpp"
#include "axis_controller.h"

using namespace GraphicsMath;

namespace PostureController {

static constexpr float32_t ANGLE_BASE_NECK   = 67.65f;  // 基座旋转轴到颈部旋转轴的夹角
static constexpr float32_t ANGLE_NECK_NOZZLE = 22.35f;  // 颈部旋转轴到喷嘴指向的夹角

class HingeJoint {
public:

static HingeJoint& getInstance(void); // 获取单例实例

// 删除拷贝构造函数和赋值操作符
HingeJoint(const HingeJoint&) = delete;
HingeJoint& operator=(const HingeJoint&) = delete;

void init_posture(void);
void init_axis(axis_handle_t* base_axis, axis_handle_t* neck_axis);
bool is_initialization_complete(void);

void update_axis(void);
void update_posture(void);

float get_base_angle_rad(void) const {return deg2rad(base_angle_deg_);}
float get_neck_angle_rad(void) const {return deg2rad(neck_angle_deg_);}
float get_base_angle_deg(void) const {return base_angle_deg_;}
float get_neck_angle_deg(void) const {return neck_angle_deg_;}
SphericalCoord get_nozzle_spherical_coord(void) const {return nozzle_spherical_coord_;}
Vector3 get_neck_axis_in_global(void) const;

float slove_neck_angle_for_nozzle_elevation(float target_elevation_deg);
float calculate_base_compensation_velocity(float neck_velocity_dps);

private:
// 私有构造函数
HingeJoint() = default;
~HingeJoint();

// 初始姿态相关配置
Vector3 base_rotate_axis_in_global_;           // 全局坐标系中基座旋转轴指向
Vector3 neck_rotate_axis_in_neck_;             // 颈部旋转轴在基座坐标系中的指向
Vector3 nozzle_vector_in_neck_;                // 颈部坐标系中喷嘴指向

Quaternion base_to_neck_q_, neck_to_nozzle_q_; // 基座到喷嘴的旋转四元数
Quaternion nozzle_to_neck_q_, neck_to_base_q_; // 喷嘴到基座的旋转四元数

// 运动、角度限位
struct {
    float32_t base_min_angle;
    float32_t base_max_angle;
    float32_t base_max_speed;
    float32_t neck_min_angle;
    float32_t neck_max_angle;
    float32_t neck_max_speed;
} limits_;

// 轴控制器句柄
axis_handle_t *base_axis_handle_, *neck_axis_handle_;

// 状态成员
float32_t base_angle_deg_ = 0.0f, neck_angle_deg_ = 0.0f; // 当前角度(角度制)
SphericalCoord nozzle_spherical_coord_; // 喷嘴指向球坐标系表示

};

}


#endif // __POSTURE_H
