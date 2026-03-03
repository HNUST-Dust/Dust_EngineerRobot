#pragma once

#include "topic.hpp"

namespace orb {

// 底盘控制指令：期望底盘坐标系速度 (m/s 或你的工程单位) + 角速度(旋转)
// 约定：该 topic 只表达“想让底盘怎么动”，不关心电机/逆解/控制细节。
struct ChassisCmd {
    float vx = 0.0f;  // +x
    float vy = 0.0f;  // +y
    float wz = 0.0f;  // +旋转

    // 使能位：用于模式切换时快速停底盘并清 PID
    bool enable = false;
};

inline Topic<ChassisCmd> chassis_cmd;

} // namespace orb
