#pragma once

#include "topic.hpp"

namespace orb {

// 底盘状态：用于 Robot/DebugTools 等读取，不再直接访问底盘内部成员。
struct ChassisState {
    bool enable = false;

    // 当前采用的控制指令（缓存）
    float cmd_vx = 0.0f;
    float cmd_vy = 0.0f;
    float cmd_wz = 0.0f;

    // 逆解后的四轮目标角速度（rad/s）
    float target_omega_1 = 0.0f;
    float target_omega_2 = 0.0f;
    float target_omega_3 = 0.0f;
    float target_omega_4 = 0.0f;

    // 四轮实际角速度（rad/s）
    float now_omega_1 = 0.0f;
    float now_omega_2 = 0.0f;
    float now_omega_3 = 0.0f;
    float now_omega_4 = 0.0f;
};

inline Topic<ChassisState> chassis_state;

} // namespace orb
