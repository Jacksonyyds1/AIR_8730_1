#include <cmath>
#include <algorithm>

#include "logger/logger.h"
#include "axis_hal/axis_controller.h"
#include "axis_hal/axis_motion_control.h"
#include "hinge_joint.hpp"

namespace PostureController
{

HingeJoint &HingeJoint::getInstance(void)
{
    static HingeJoint instance;
    return instance;
}

void HingeJoint::init_posture(void)
{
    // 1. 定义基座和颈部的旋转轴
    base_rotate_axis_in_global_ = Vector3(0.0f, 0.0f, 1.0f); // 基座绕全局Z轴旋转
    neck_rotate_axis_in_neck_   = Vector3(0.0f, 0.0f, 1.0f); // 颈部绕其局部Z轴旋转

    // 2. 定义坐标系之间的静态变换
    // 定义绕Y轴的旋转
    const Vector3 ROTATION_AXIS_FOR_TILT = Vector3(0.0f, 1.0f, 0.0f);

    // 创建从 基座坐标系 -> 颈部坐标系 的旋转
    base_to_neck_q_ = Quaternion::fromAxisAngle(ROTATION_AXIS_FOR_TILT, ANGLE_BASE_NECK);
    // 创建从 颈部坐标系 -> 基座坐标系 的旋转
    neck_to_base_q_ = base_to_neck_q_.conjugate();

    // 创建从 颈部坐标系 -> 喷嘴方向 的旋转
    neck_to_nozzle_q_ = Quaternion::fromAxisAngle(ROTATION_AXIS_FOR_TILT, ANGLE_NECK_NOZZLE);
    // 创建从 喷嘴方向 -> 颈部坐标系 的旋转
    nozzle_to_neck_q_ = neck_to_nozzle_q_.conjugate();

    // 3. 计算喷嘴在颈部坐标系中的初始指向
    // 喷嘴的初始方向是颈部的旋转轴(Z轴)经过 neck_to_nozzle_q 旋转后的结果
    SphericalCoord nozzle_direction_in_neck_sphere_(0, 90.0f - ANGLE_NECK_NOZZLE);
    nozzle_vector_in_neck_ = nozzle_direction_in_neck_sphere_.toVector();

    // 4. 计算最终的初始姿态 (在全局/基座坐标系中)
    // 将“在颈部坐标系中的喷嘴方向”通过 base_to_neck_q 变换到基座坐标系
    Vector3 initial_nozzle_vector_in_global_ = base_to_neck_q_.rotateVector(nozzle_vector_in_neck_);

    // 5. 将最终方向转换为球坐标并初始化关节角度
    nozzle_spherical_coord_ = SphericalCoord(initial_nozzle_vector_in_global_);
    base_angle_deg_ = 0.0f; // 初始化基座角度
    neck_angle_deg_ = 0.0f; // 初始化颈部角度

    // 步骤5: 消除微小误差导致的角度超出设计范围
    if(nozzle_spherical_coord_.elevation <= 0.0f)
    {
        nozzle_spherical_coord_.elevation = 0.0f;
    }
    else if(nozzle_spherical_coord_.elevation >= 90.0f - (ANGLE_BASE_NECK - ANGLE_NECK_NOZZLE))
    {
        nozzle_spherical_coord_.elevation = 90.0f - (ANGLE_BASE_NECK - ANGLE_NECK_NOZZLE);
    }

    LOGI("Initial Nozzle Direction (Global): [%f, %f, %f]",
            initial_nozzle_vector_in_global_.x, initial_nozzle_vector_in_global_.y, initial_nozzle_vector_in_global_.z);
    LOGI("Initial Nozzle Spherical Coord: Azimuth: %f, Elevation: %f",
            nozzle_spherical_coord_.azimuth, nozzle_spherical_coord_.elevation);
}

void HingeJoint::init_axis(axis_handle_t *base_axis, axis_handle_t *neck_axis)
{
    base_axis_handle_ = base_axis;
    neck_axis_handle_ = neck_axis;

    limits_.base_min_angle = base_axis->position.min_angle_limit;
    limits_.base_max_angle = base_axis->position.max_angle_limit;
    limits_.neck_min_angle = neck_axis->position.min_angle_limit;
    limits_.neck_max_angle = neck_axis->position.max_angle_limit;

    axis_start_initialization(base_axis_handle_);
    axis_start_initialization(neck_axis_handle_);
}

bool HingeJoint::is_initialization_complete(void)
{
    return axis_is_initialization_complete(base_axis_handle_) &&
            axis_is_initialization_complete(neck_axis_handle_);
}

void HingeJoint::update_axis(void)
{
    encoder_process_samples();
    axis_update(base_axis_handle_);
    axis_update(neck_axis_handle_);
}

void HingeJoint::update_posture(void)
{
    axis_update_position_from_encoder(base_axis_handle_);
    axis_update_position_from_encoder(neck_axis_handle_);

    Quaternion neck_rotation_q = Quaternion::fromAxisAngle(neck_rotate_axis_in_neck_, neck_angle_deg_);
    Quaternion base_rotation_q = Quaternion::fromAxisAngle(base_rotate_axis_in_global_, base_angle_deg_);

    Vector3 nozzle_vector;

    // 步骤1: 喷嘴指向绕颈部旋转轴旋转
    nozzle_vector = neck_rotation_q.rotateVector(nozzle_vector_in_neck_);

    // 步骤2: 喷嘴指向变换到基座坐标系
    nozzle_vector = base_to_neck_q_.rotateVector(nozzle_vector);

    // 步骤3: 喷嘴指向绕基座旋转轴旋转
    nozzle_vector = base_rotation_q.rotateVector(nozzle_vector);

    // 步骤4: 更新喷嘴的球坐标表示(将1e-6f的误差归零)
    nozzle_spherical_coord_ = SphericalCoord(nozzle_vector);

    // 步骤5: 消除微小误差导致的角度超出设计范围
    if(nozzle_spherical_coord_.elevation <= 0.0f)
    {
        nozzle_spherical_coord_.elevation = 0.0f;
    }
    else if(nozzle_spherical_coord_.elevation >= 90.0f - (ANGLE_BASE_NECK - ANGLE_NECK_NOZZLE))
    {
        nozzle_spherical_coord_.elevation = 90.0f - (ANGLE_BASE_NECK - ANGLE_NECK_NOZZLE);
    }
}

Vector3 HingeJoint::get_neck_axis_in_global(void) const
{
    Vector3 neck_vector = base_to_neck_q_.rotateVector(neck_rotate_axis_in_neck_);
    Quaternion base_rotate_q = Quaternion::fromAxisAngle(base_rotate_axis_in_global_, base_angle_deg_);
    return base_rotate_q.rotateVector(neck_vector);
}

/**
 * @brief 根据nozzle仰角计算neck角度
 * @param target_elevation_deg 目标仰角（度）
 * @return neck物理角度（度，0-180°），如果无解返回当前角度
 */
float HingeJoint::slove_neck_angle_for_nozzle_elevation(float target_elevation_deg)
{
    // 机械参数转换为弧度
    float alpha_deg = ANGLE_BASE_NECK;     // neck轴与Z轴夹角
    float beta_deg = ANGLE_NECK_NOZZLE;    // nozzle与neck轴夹角
    float target_elev_rad = deg2rad(target_elevation_deg);

    float sin_target = arm_sin_f32(target_elev_rad);
    float cos_alpha, sin_alpha;
    arm_sin_cos_f32(alpha_deg, &cos_alpha, &sin_alpha);

    float cos_beta, sin_beta;
    arm_sin_cos_f32(beta_deg, &cos_beta, &sin_beta);

    // 计算cos(θ)，几何解算
    float numerator = sin_target - cos_alpha * cos_beta;
    float denominator = sin_alpha * sin_beta;

    if(std::abs(denominator) < 1e-10f)
    {
        // 分母接近零，使用当前角度
        LOGW("No valid neck angle solution for target elevation: %f", target_elevation_deg);
        return neck_angle_deg_;
    }

    float cos_theta = numerator / denominator;

    // 检查cos(θ)是否在有效范围内，考虑浮点精度误差
    const float epsilon = 1e-6f;  // 允许的浮点误差
    if(cos_theta < -1.0f - epsilon || cos_theta > 1.0f + epsilon)
    {
        // 超出物理可达范围，使用当前角度
        LOGW("Cosine of theta out of bounds for target elevation: %f", target_elevation_deg);
        return neck_angle_deg_;
    }

    // 钳制到有效范围，处理浮点精度问题
    if(cos_theta < -1.0f)
    {
        cos_theta = -1.0f;
    }
    else if(cos_theta > 1.0f)
    {
        cos_theta = 1.0f;
    }

    // 计算θ（两个几何解）
    float theta_rad = std::acosf(cos_theta);
    float theta1_geometric = rad2deg(theta_rad);    // 0-180°范围内的正解
    float theta2_geometric = rad2deg(-theta_rad);   // 负角度解

    // 将负角度转换为正角度（0-360°范围）
    if(theta2_geometric < 0.0f)
    {
        theta2_geometric += 360.0f;
    }

    // 限制在0-180°范围内（只考虑acos的主值范围）
    if(theta2_geometric > 180.0f)
    {
        theta2_geometric = 360.0f - theta2_geometric;
    }

    // 应用角度翻转，将几何解转换为物理角度
    // 几何解：180°=水平，0°=最大仰角
    // 物理角度：0°=水平，180°=最大仰角
    float theta1_physical = 180.0f - theta1_geometric;
    float theta2_physical = 180.0f - theta2_geometric;

    // 应用neck轴物理限位（0°到180°）
    bool theta1_valid = (theta1_physical >= 0.0f && theta1_physical <= 180.0f);
    bool theta2_valid = (theta2_physical >= 0.0f && theta2_physical <= 180.0f);

    if(!theta1_valid && !theta2_valid)
    {
        // 无有效解，使用当前角度
        LOGW("No valid neck angle solution for target elevation: %f", target_elevation_deg);
        return neck_angle_deg_;
    }

    // 选择距离当前角度更近的解
    float current_neck = neck_angle_deg_;

    if(theta1_valid && theta2_valid)
    {
        float diff1 = std::abs(theta1_physical - current_neck);
        float diff2 = std::abs(theta2_physical - current_neck);
        return (diff1 < diff2) ? theta1_physical : theta2_physical;
    }
    else if(theta1_valid)
    {
        return theta1_physical;
    }
    else
    {
        return theta2_physical;
    }
}

/**
 * @brief 根据颈部运动计算基座所需的补偿角速度，以维持喷嘴方位角不变
 * @return float 基座需要设置的补偿角速度（度/秒）
 */
float HingeJoint::calculate_base_compensation_velocity(float neck_velocity_dps)
{
    // 如果颈部没有在动，则不需要补偿
    if (std::abs(neck_velocity_dps) < 1e-3f) {
        return 0.0f;
    }

    // 1. 获取完成正向运动学计算所需的最新状态
    // 注意：这里依赖于 axis_system 在调用此函数前已经被 update_posture() 更新过
    Vector3 nozzle_vec_global = nozzle_spherical_coord_.toVector();
    Vector3 neck_axis_global = get_neck_axis_in_global();

    // 2. 计算颈部旋转引起的喷嘴瞬时线速度 (v = ω × r)
    // ω: 颈部角速度向量 (方向是颈部旋转轴，大小是角速度)
    // r: 旋转半径向量 (从原点到喷嘴的向量)
    float neck_omega_rad = deg2rad(neck_velocity_dps);
    Vector3 neck_angular_velocity_vec = neck_axis_global * neck_omega_rad;
    Vector3 nozzle_tangential_velocity = neck_angular_velocity_vec * nozzle_vec_global; // 叉乘

    // 3. 将线速度投影到基座旋转的XY平面
    Vector3 velocity_in_xy_plane(nozzle_tangential_velocity.x, nozzle_tangential_velocity.y, 0.0f);

    // 4. 计算喷嘴位置在XY平面上的投影半径
    Vector3 nozzle_pos_in_xy_plane(nozzle_vec_global.x, nozzle_vec_global.y, 0.0f);
    float radius_in_xy_plane = nozzle_pos_in_xy_plane.magnitude();

    // 如果半径接近于0 (喷嘴几乎垂直向上/下)，无法进行有效补偿，返回0
    if (radius_in_xy_plane < 1e-3f) {
        return 0.0f;
    }

    // 5. 计算补偿角速度 (ω = v / r)
    float linear_velocity_mag = velocity_in_xy_plane.magnitude();
    float base_compensation_omega_rad = linear_velocity_mag / radius_in_xy_plane;

    // 6. 确定补偿方向 (符号)
    // 通过 p_xy 和 v_xy 的叉乘的z分量来判断 v_xy 产生的旋转方向
    // (p_xy × v_xy).z > 0 表示逆时针运动，需要一个负的（顺时针）补偿
    float sign_check = nozzle_pos_in_xy_plane.x * velocity_in_xy_plane.y - nozzle_pos_in_xy_plane.y * velocity_in_xy_plane.x;
    if (sign_check > 0) {
        base_compensation_omega_rad = -base_compensation_omega_rad;
    }
    
    // 7. 转换为度/秒并返回
    return rad2deg(base_compensation_omega_rad);
}

HingeJoint::~HingeJoint()
{
    axis_destroy(base_axis_handle_);
    axis_destroy(neck_axis_handle_);
}

}
