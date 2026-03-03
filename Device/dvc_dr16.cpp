#include "dvc_dr16.h"

#include "../communication_topic/rc_control_topics.hpp"

#include <algorithm>
#include <cstdint>
#include <cstring>

void Dr16::Init(BspUartHandle uart)
{
    uart_ = uart;
    started_ = false;

    // 保证 rc_control 上电初值为 0：不依赖拨杆量/外部模块判定
    {
        orb::RcControl rc{};  // 全字段默认 0
        orb::rc_control.publish(rc);
    }
}

void Dr16::RxCpltCallback(uint8_t* buffer, uint16_t length)
{
    constexpr uint16_t kFrameLen = 18;
    if (!buffer || length < kFrameLen) return;

    Dr16RecviedRawData tmp{};

    // 摇杆通道
    tmp.channel0 = static_cast<int16_t>(((buffer[0] | (buffer[1] << 8)) & 0x07FF) - 1024);
    tmp.channel1 = static_cast<int16_t>(((buffer[1] >> 3 | (buffer[2] << 5)) & 0x07FF) - 1024);
    tmp.channel2 = static_cast<int16_t>(((buffer[2] >> 6 | (buffer[3] << 2) | (buffer[4] << 10)) & 0x07FF) - 1024);
    tmp.channel3 = static_cast<int16_t>(((buffer[4] >> 1 | (buffer[5] << 7)) & 0x07FF) - 1024);

    // 拨杆
    tmp.switch1 = static_cast<uint8_t>(((buffer[5] >> 4) & 0x0C) >> 2);
    tmp.switch2 = static_cast<uint8_t>((buffer[5] >> 4) & 0x03);

    // 鼠标
    tmp.mouse.x = static_cast<int16_t>(buffer[6] | (buffer[7] << 8));
    tmp.mouse.y = static_cast<int16_t>(buffer[8] | (buffer[9] << 8));
    tmp.mouse.z = static_cast<int16_t>(buffer[10] | (buffer[11] << 8));
    tmp.mouse.l = buffer[12];
    tmp.mouse.r = buffer[13];

    // 键盘
    tmp.keyboard.key_code = static_cast<uint16_t>(buffer[14] | (buffer[15] << 8));

    // 拨轮
    tmp.pulley_wheel = static_cast<int16_t>((static_cast<int16_t>(buffer[16] | (buffer[17] << 8))) - 1024);

    std::memcpy(&pre_uart_rx_data_, &tmp, sizeof(Dr16RecviedRawData));

    // 归一化到 [-1,1]
    auto norm = [&](int16_t ch) {
        float v = static_cast<float>(ch) / kRockerNum;
        return std::clamp(v, -1.0f, 1.0f);
    };

    const float ch0 = norm(tmp.channel0);
    const float ch1 = norm(tmp.channel1);
    const float ch2 = norm(tmp.channel2);
    const float ch3 = norm(tmp.channel3);
    const float wheel = norm(tmp.pulley_wheel);

    // stick alias
    const float left_stick_x = ch2;
    const float left_stick_y = ch3;
    const float right_stick_x = ch0;
    const float right_stick_y = ch1;

    // 遥控端累计量：本工程约定所有 angle/distance 都在遥控端做积累后发布
    static float elbow_pitch_acc = 0.0f;
    static float elbow_yaw_acc   = 0.0f;
    static float wrist_pitch_acc = 0.0f;
    static float wrist_yaw_acc   = 0.0f;
    static float gantry_z_acc    = 0.0f;

    constexpr float kElbowAccStep  = 1.0f;
    constexpr float kWristAccStep  = 1.0f;
    constexpr float kGantryAccStep = 1.0f;
    constexpr float kGantryZLimit  = 100.0f; // 与 app_gantry.h Z_AXIS_DISTANCE_LIMIT 对齐

    orb::RcControl info{};

    switch (tmp.switch1) {
    case 1: {
        // chassis
        info.chassis_x_speed = +left_stick_x;
        info.chassis_y_speed = -left_stick_y;
        info.chassis_rotation_speed = wheel;
        break;
    }
    case 2: {
        // arm
        info.arm_claw_angle = -right_stick_x;

        // wrist: 积累
        wrist_yaw_acc += wheel * kWristAccStep;
        wrist_pitch_acc += right_stick_y * kWristAccStep;
        info.arm_wrist_yaw_angle = wrist_yaw_acc;
        info.arm_wrist_pitch_angle = wrist_pitch_acc;

        // elbow: 积累
        elbow_pitch_acc += left_stick_y * kElbowAccStep;
        elbow_yaw_acc += left_stick_x * kElbowAccStep;
        info.arm_elbow_pitch_angle = elbow_pitch_acc;
        info.arm_elbow_yaw_angle = elbow_yaw_acc;
        break;
    }
    case 3: {
        // gantry
        // X/Y 直接输出摇杆值（Gantry 侧作为速度目标）
        info.gantry_x_axis_distance = left_stick_y * kGantryAccStep;
        info.gantry_y_axis_distance = left_stick_x * kGantryAccStep;
        // Z 在 DR16 侧积累，发布目标角度
        gantry_z_acc += right_stick_y * kGantryAccStep;
        if (gantry_z_acc >  kGantryZLimit) gantry_z_acc =  kGantryZLimit;
        if (gantry_z_acc < -kGantryZLimit) gantry_z_acc = -kGantryZLimit;
        info.gantry_z_axis_distance = gantry_z_acc;
        break;
    }
    default:
        break;
    }

    orb::rc_control.publish(info);
}

