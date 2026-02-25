
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

void VT02::Init(BspUartHandle uart, orb::UartPort vt02_port)
{
	if (uart == nullptr) {
		return;
	}
	if (started_) {
		return;
	}

	uart_ = uart;
	vt02_port_ = vt02_port;
	started_ = true;
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

	const float mouse_x = (static_cast<float>(raw_mouse_x) / 32768.0f) * kMouseSensitivityX;
	mouse_y += (static_cast<float>(raw_mouse_y) / 32768.0f) * kMouseSensitivityY;
	mouse_y = std::clamp(mouse_y, -1.0f, 1.0f);

	const uint8_t yaw_u8 = clamp_to_u8((mouse_x + 1.0f) * 127.5f);
	const uint8_t pitch_u8 = clamp_to_u8((mouse_y + 1.0f) * 127.5f);

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

	const float wheel_norm = static_cast<float>(pulley_wheel) / 32768.0f;
	const uint8_t wheel_u8 = clamp_to_u8((wheel_norm + 1.0f) * 127.5f);

	orb::RcControl info{};
	info.yaw_angle = yaw_u8;
	info.pitch_angle = pitch_u8;
	info.chassis_speed_x = clamp_to_u8(target_speed_x);
	info.chassis_speed_y = clamp_to_u8(target_speed_y);
	info.chassis_rotation = wheel_u8;
    info.chassis_spin = (tmp.keyboard.bit.SHIFT != 0) ? orb::ChassisSpin::ChassisSpinCW
													 : orb::ChassisSpin::ChassisSpinNone;
	info.super_cap = (tmp.keyboard.bit.E != 0) ? orb::SuperCap::SuperCapDischarge
											   : orb::SuperCap::SuperCapCharge;
	info.auto_aim = (mouse_right != 0) ? orb::AutoAim::AutoAimOn : orb::AutoAim::AutoAimOff;
	info.gimbal_set_zero = (tmp.keyboard.bit.B != 0) ? orb::GimbalSetZero::GimbalSetZeroOn
													 : orb::GimbalSetZero::GimbalSetZeroOff;
	info.friction_wheel = (tmp.keyboard.bit.F != 0) ? orb::FrictionWheel::FrictionWheelOn
													: orb::FrictionWheel::FrictionWheelOff;
	info.shoot = (mouse_left != 0) ? orb::Shoot::ShootOn : orb::Shoot::ShootOff;
	info.eject = (tmp.keyboard.bit.R != 0) ? orb::Eject::EjectOn : orb::Eject::EjectOff;

	orb::rc_control.publish(info);
}


