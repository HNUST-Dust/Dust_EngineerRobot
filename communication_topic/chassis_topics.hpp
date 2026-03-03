#pragma once

#include "topic.hpp"

// 底盘状态 Topic：发布四个轮子的实时角速度(rad/s)
struct ChassisWheelOmega {
    float omega_1_rad_s = 0.0f;
    float omega_2_rad_s = 0.0f;
    float omega_3_rad_s = 0.0f;
    float omega_4_rad_s = 0.0f;
};

// 只关心最新值：用 Topic 即可
inline Topic<ChassisWheelOmega> chassis_wheel_omega;
