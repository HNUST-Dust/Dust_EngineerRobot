#pragma once
#include <cstdint>

#include "topic.hpp"

namespace orb {

// 与 Robot.cpp 当前遥控逻辑对齐：Robot 直接使用这些字段。
// 约定：各 stick/wheel 为 [-1,1]，开关为原始 SBus 值 (1/2/3)
struct RcControl {
    float chassis_x_speed = 0.0f;
    float chassis_y_speed = 0.0f;
    float chassis_rotation_speed = 0.0f;

    float arm_elbow_pitch_angle = 0.0f;
    float arm_elbow_yaw_angle = 0.0f;
    float arm_wrist_pitch_angle = 0.0f;
    float arm_wrist_yaw_angle = 0.0f;
    float arm_claw_angle = 0.0f;

    float gantry_x_axis_distance = 0.0f;
    float gantry_y_axis_distance = 0.0f;
    float gantry_z_axis_distance = 0.0f;

    float gimbal_pitch_angle = 0.0f;
};

inline Topic<RcControl> rc_control;

} // namespace orb