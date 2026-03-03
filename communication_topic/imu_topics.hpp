#pragma once
#include <cstdint>
#include "topic.hpp"

namespace orb {
struct ImuData {
    float q[4]; // 四元数
    float yaw_angle;
    float pitch_angle;
    float yaw_omega;
    float pitch_omega;
};
inline Topic<ImuData> imu_data;
}