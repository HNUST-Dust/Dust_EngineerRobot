#pragma once
#include <cstdint>

#include "topic.hpp"

namespace orb {

enum class ClawsStatus : uint8_t {
    Open = 1,
    Close = 0,
};

struct RcControl {
    uint8_t chassis_speed_x;          // 平移方向：前、后、左、右
    uint8_t chassis_speed_y;          // 底盘移动总速度
    uint8_t chassis_rotation;         // 旋转速度
    uint8_t gimbal_pitch_angle;       // 云台俯仰角度
    uint8_t gantry_x_speed;           // 龙门架X轴速度
    uint8_t gantry_y_speed;           // 龙门架Y轴速度
    uint8_t gantry_z_speed;           // 龙门架Z轴速度
    ClawsStatus claws_status;         // 爪子状态：开/关
    uint8_t wrist_pitch_speed;        // 手腕俯仰速度
    uint8_t wrist_roll_speed;         // 手腕翻转速度
    uint8_t elbow_pitch_speed;        // 肘部俯仰速度
    uint8_t elbow_yaw_speed;          // 肘部扭转速度
};

inline Topic<RcControl> rc_control;
}// namespace orb