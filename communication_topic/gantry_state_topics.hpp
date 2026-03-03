#pragma once

#include "topic.hpp"

namespace orb {

struct GantryState {
    bool enable = false;

    // 缓存的输入
    float in_x_speed = 0.0f;
    float in_y_speed = 0.0f;
    float in_z_speed = 0.0f;

    // X/Y 目标速度（速度控制模式）
    float x_target_omega = 0.0f;
    float y_target_omega = 0.0f;
    // Z 目标角度（位置控制模式）
    float z_target_angle = 0.0f;

    float virtual_z_distance = 0.0f;

    // X/Y 当前速度（rad/s）
    float x_left_now_omega  = 0.0f;
    float x_right_now_omega = 0.0f;
    float y_now_omega       = 0.0f;

    // Z 当前角度与速度
    float z_left_now_angle  = 0.0f;
    float z_right_now_angle = 0.0f;
    float z_left_omega      = 0.0f;
    float z_left_total_angle = 0.0f;
    float z_left_torque     = 0.0f;
};

inline Topic<GantryState> gantry_state;

} // namespace orb
