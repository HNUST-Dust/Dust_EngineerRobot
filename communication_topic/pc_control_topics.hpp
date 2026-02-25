#pragma once
#include <cstdint>
#include "topic.hpp"
namespace orb {

struct PcControl {
    uint8_t shoot_switch;         // 发射控制
    uint8_t auto_aim_flag;        // 自动瞄准状态
    float yaw_angle;
    float yaw_omega;
    float yaw_torque;
    float pitch_angle;
    float pitch_omega;
    float pitch_torque;
};

inline Topic<PcControl> pc_control;

} // namespace orb