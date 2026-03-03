#pragma once

#include "topic.hpp"

namespace orb {

struct ArmState {
    bool enable = false;

    // 缓存最新输入（便于 DebugTools 侧观测）
    // arm_input_topics.hpp 已移除，因此这里保留最常用的观测字段，避免交叉依赖。
    float last_cmd_claws = 0.0f;
    float last_cmd_wrist_left = 0.0f;
    float last_cmd_wrist_right = 0.0f;
    float last_cmd_elbow_pitch = 0.0f;
    float last_cmd_elbow_yaw = 0.0f;

    // 虚拟角（模块内部累加后的结果，单位 rad）
    float claws_virtual_angle = 0.0f;
    float wrist_left_virtual_angle = 0.0f;
    float wrist_right_virtual_angle = 0.0f;
    float elbow_pitch_virtual_angle = 0.0f;
    float elbow_yaw_virtual_angle = 0.0f;

    // 反馈（best-effort；若电机对象为空则为 0）
    float claws_now_angle = 0.0f;
    float claws_now_omega = 0.0f;

    float elbow_pitch_now_angle = 0.0f;
    float elbow_pitch_now_omega = 0.0f;

    float elbow_yaw_now_angle = 0.0f;
    float elbow_yaw_now_omega = 0.0f;

    float wrist_left_now_angle = 0.0f;
    float wrist_left_now_omega = 0.0f;

    float wrist_right_now_angle = 0.0f;
    float wrist_right_now_omega = 0.0f;

    // 输出（用于调试）
    float claws_torque = 0.0f;
    float elbow_pitch_torque = 0.0f;
    float elbow_yaw_torque = 0.0f;
    float wrist_left_current = 0.0f;
    float wrist_right_current = 0.0f;
};

inline Topic<ArmState> arm_state;

}  // namespace orb
