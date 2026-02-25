#pragma once

#include <cstdint>

namespace motor_ids {

// CAN1
static constexpr uint32_t kMotorZAxisRight = 0x000; // 龙门架Z轴右电机
static constexpr uint32_t kMotorChassis1 = 0x201; // 底盘电机1
static constexpr uint32_t kMotorChassis2 = 0x202; // 底盘电机2
static constexpr uint32_t kMotorChassis3 = 0x203; // 底盘电机3
static constexpr uint32_t kMotorChassis4 = 0x204; // 底盘电机4

// CAN2
static constexpr uint32_t kMotorZAxisLeft = 0x000; // 龙门架Z轴左电机
static constexpr uint32_t kMotorXAxisLeft = 0x201; // 龙门架X轴左电机
static constexpr uint32_t kMotorXAxisRight = 0x202; // 龙门架X轴右电机

// CAN3
static constexpr uint32_t kMotorYAxis = 0x201; // 龙门架Y轴电机
static constexpr uint32_t kMotorWristLeft = 0x202; // 机械臂手腕关节左电机
static constexpr uint32_t kMotorWristRight = 0x203; // 机械臂手腕关节右电机
static constexpr uint32_t kMotorClaws = 0x11; // 机械臂手爪电机
static constexpr uint32_t kMotorElbowYaw = 0x12; // 机械臂肘部关节偏航电机
static constexpr uint32_t kMotorElbowPitch = 0x13; // 机械臂肘部关节俯仰电机
} // namespace motor_ids
