#pragma once

#include "topic.hpp"

namespace orb {

// Minimal IMU feedback published by local INS (used by cascade motor loops).
// Naming keeps compatibility with existing motor code.
struct McuImu {
    float yaw_total_angle_f;
    float yaw_omega_f;
    float pitch_f;
    float pitch_omega_f;
};

inline Topic<McuImu> mcu_imu;

} // namespace orb
