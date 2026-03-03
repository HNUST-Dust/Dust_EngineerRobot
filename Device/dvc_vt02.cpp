#include "dvc_vt02.h"

#include "../communication_topic/rc_control_topics.hpp"

#include <algorithm>
#include <cstdint>
#include <cstring>

#include "CRC.h"

namespace {

inline uint8_t clamp_to_u8(float v)
{
	v = std::clamp(v, 0.0f, 255.0f);
	return static_cast<uint8_t>(v);
}

}  // namespace

// 全局状态：保持与 dvc_vt03 一致的“鼠标 Y 累积”以及键盘速度斜坡
static float mouse_y = 0.0f;
static float target_speed_x = 127.0f;
static float target_speed_y = 127.0f;

void VT02::Init(BspUartHandle uart)
{
	if (uart == nullptr) {
		return;
	}
	if (started_) {
		return;
	}

	uart_ = uart;
	started_ = false;

	// 保证 rc_control 上电初值为 0
	{
		orb::RcControl rc{};
		orb::rc_control.publish(rc);
	}

	// 发布一次全 0 的 RcControl，保证未收到键鼠/帧时输出为 0
	{
		orb::RcControl rc{};
		rc.chassis_x_speed = 0.0f;
		rc.chassis_y_speed = 0.0f;
		rc.chassis_rotation_speed = 0.0f;
		rc.arm_claw_angle = 0.0f;
		rc.arm_wrist_yaw_angle = 0.0f;
		rc.arm_wrist_pitch_angle = 0.0f;
		rc.arm_elbow_pitch_angle = 0.0f;
		rc.arm_elbow_yaw_angle = 0.0f;
		rc.gantry_x_axis_distance = 0.0f;
		rc.gantry_y_axis_distance = 0.0f;
		rc.gantry_z_axis_distance = 0.0f;
		rc.gimbal_pitch_angle = 0.0f;
		orb::rc_control.publish(rc);
	}
}

void VT02::RxCpltCallback(uint8_t* buffer, uint16_t length)
{
    if (!buffer || length < sizeof(VT02RecviedRawData)) return;

    if (!verify_crc16_check_sum(buffer, static_cast<uint32_t>(length))) {
        return;
    }

    const int16_t raw_mouse_x = static_cast<int16_t>((static_cast<uint16_t>(buffer[7]) << 8) | buffer[8]);
    const int16_t raw_mouse_y = static_cast<int16_t>((static_cast<uint16_t>(buffer[9]) << 8) | buffer[10]);
    const int16_t raw_mouse_z = static_cast<int16_t>((static_cast<uint16_t>(buffer[11]) << 8) | buffer[12]);
    (void)raw_mouse_z;

    const uint8_t mouse_left = buffer[13];
    const uint8_t mouse_right = buffer[14];

    const uint16_t key_code = static_cast<uint16_t>(buffer[15]) | (static_cast<uint16_t>(buffer[16]) << 8);
    VT02RecviedRawData tmp{};
    tmp.keyboard.key_code = key_code;

    int16_t pulley_wheel = 0;
    if (length >= 19) {
        pulley_wheel = static_cast<int16_t>((static_cast<uint16_t>(buffer[17]) << 8) | buffer[18]);
    }

    std::memcpy(&pre_uart_rx_data_, &tmp, sizeof(VT02RecviedRawData));

    // 将 VT02 的键鼠控制映射到统一的 RcControl：
    // - right stick: 鼠标（用于云台）
    // - left stick: 键盘 W/S/A/D 的缓启动/缓停止速度
    // - wheel: 拨轮
    const float mouse_x = (static_cast<float>(raw_mouse_x) / 32768.0f) * kMouseSensitivityX;
    mouse_y += (static_cast<float>(raw_mouse_y) / 32768.0f) * kMouseSensitivityY;
    mouse_y = std::clamp(mouse_y, -1.0f, 1.0f);

    constexpr float kTargetMin = 0.0f;
    constexpr float kTargetMax = 255.0f;
    constexpr float kTargetCenter = 127.0f;
    constexpr float kStartStep = 1.0f;
    constexpr float kStopStep = 1.0f;

    const bool w_pressed = (tmp.keyboard.bit.W != 0);
    const bool s_pressed = (tmp.keyboard.bit.S != 0);
    const bool a_pressed = (tmp.keyboard.bit.A != 0);
    const bool d_pressed = (tmp.keyboard.bit.D != 0);

    if (w_pressed && !s_pressed) {
        target_speed_x = std::min(target_speed_x + kStartStep, kTargetMax);
    } else if (s_pressed && !w_pressed) {
        target_speed_x = std::max(target_speed_x - kStartStep, kTargetMin);
    } else {
        if (target_speed_x < kTargetCenter) {
            target_speed_x = std::min(target_speed_x + kStopStep, kTargetCenter);
        } else if (target_speed_x > kTargetCenter) {
            target_speed_x = std::max(target_speed_x - kStopStep, kTargetCenter);
        }
    }

    if (d_pressed && !a_pressed) {
        target_speed_y = std::min(target_speed_y + kStartStep, kTargetMax);
    } else if (a_pressed && !d_pressed) {
        target_speed_y = std::max(target_speed_y - kStartStep, kTargetMin);
    } else {
        if (target_speed_y < kTargetCenter) {
            target_speed_y = std::min(target_speed_y + kStopStep, kTargetCenter);
        } else if (target_speed_y > kTargetCenter) {
            target_speed_y = std::max(target_speed_y - kStopStep, kTargetCenter);
        }
    }

    const float left_stick_x = std::clamp((target_speed_y - kTargetCenter) / 127.0f, -1.0f, 1.0f);
    const float left_stick_y = std::clamp((target_speed_x - kTargetCenter) / 127.0f, -1.0f, 1.0f);

    const float wheel_norm = std::clamp(static_cast<float>(pulley_wheel) / 32768.0f, -1.0f, 1.0f);

    // 遥控端累计量：所有 angle/distance 都做积累后发布
    static float elbow_pitch_acc = 0.0f;
    static float elbow_yaw_acc = 0.0f;
    static float wrist_pitch_acc = 0.0f;
    static float wrist_yaw_acc = 0.0f;
    static float gantry_x_acc = 0.0f;
    static float gantry_y_acc = 0.0f;
    static float gantry_z_acc = 0.0f;

    constexpr float kElbowAccStep = 1.0f;
    constexpr float kWristAccStep = 1.0f;
    constexpr float kGantryAccStep = 1.0f;

    orb::RcControl info{};

    // chassis
    info.chassis_x_speed = left_stick_x;
    info.chassis_y_speed = -left_stick_y;
    info.chassis_rotation_speed = wheel_norm;

    // arm
    info.arm_claw_angle = 0.0f;

    // wrist: 积累
    wrist_yaw_acc += wheel_norm * kWristAccStep;
    wrist_pitch_acc += std::clamp(mouse_y, -1.0f, 1.0f) * kWristAccStep;
    info.arm_wrist_yaw_angle = wrist_yaw_acc;
    info.arm_wrist_pitch_angle = wrist_pitch_acc;

    // elbow: 积累
    elbow_pitch_acc += left_stick_y * kElbowAccStep;
    elbow_yaw_acc += left_stick_x * kElbowAccStep;
    info.arm_elbow_pitch_angle = elbow_pitch_acc;
    info.arm_elbow_yaw_angle = elbow_yaw_acc;

    // gantry: 积累
    gantry_x_acc += left_stick_y * kGantryAccStep;
    gantry_y_acc += left_stick_x * kGantryAccStep;
    gantry_z_acc += std::clamp(mouse_y, -1.0f, 1.0f) * kGantryAccStep;
    info.gantry_x_axis_distance = gantry_x_acc;
    info.gantry_y_axis_distance = gantry_y_acc;
    info.gantry_z_axis_distance = gantry_z_acc;

    // gimbal
    info.gimbal_pitch_angle = 0.0f;

    (void)mouse_left;
    (void)mouse_right;

    orb::rc_control.publish(info);
}


