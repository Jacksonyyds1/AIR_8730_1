#ifndef __POSTURE_CONTROLLER_H
#define __POSTURE_CONTROLLER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include "axis_controller.h"

typedef enum {
    Posture_Controller_State_Uninitialized,     // 未初始化
    Posture_Controller_State_Initializing,      // 初始化中
    Posture_Controller_State_Idle,              // 空闲
    Posture_Controller_State_Position_Control,  // 位置控制模式
    Posture_Controller_State_Base_Oscillation,  // 基座摆动模式
    Posture_Controller_State_Elevation_Control, // 仰角控制模式
    Posture_Controller_State_Test_Mode,         // 测试模式
    Posture_Controller_State_Error              // 错误状态
} posture_controller_state_t;

typedef enum {
    Oscillation_State_Start,    // 开始摆动
    Oscillation_State_Forward,  // 向前摆动
    Oscillation_State_Backward, // 向后摆动
    Oscillation_State_Wait      // 等待状态
} oscillation_state_t;

void posture_controller_task_init(void);
bool posture_controller_is_ready(void);
bool posture_controller_move_base(float angle_deg);
bool posture_controller_set_base_oscillation_range(float range_deg);
bool posture_controller_set_elevation(float angle_deg);
bool posture_controller_set_test_mode(void);
bool posture_controller_stop(void);
float slove_neck_angle_for_nozzle_elevation(float target_elevation_deg);
float calculate_base_compensation_velocity(float neck_velocity_dps);

#ifdef __cplusplus
}
#endif

#endif // __POSTURE_CONTROLLER_H