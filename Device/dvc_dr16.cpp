
#include "dvc_dr16.h"

#include "../communication_topic/rc_control_topics.hpp"

#include <algorithm>
#include <cstdint>
#include <cstring>

namespace {

inline uint8_t clamp_to_u8(float v)
{
	v = std::clamp(v, 0.0f, 255.0f);
	return static_cast<uint8_t>(v);
}

}  // namespace

static float mouse_y = 0.0f;
static float target_speed_x = 127.0f;
static float target_speed_y = 127.0f;

void Dr16::Init(BspUartHandle uart, orb::UartPort dr16_port)
{
	if (uart == nullptr) {
		return;
	}
	if (started_) {
		return;
	}

	uart_ = uart;
	dr16_port_ = dr16_port;
	started_ = true;
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
	tmp.pulley_wheel = static_cast<int16_t>(-(static_cast<int16_t>(buffer[16] | (buffer[17] << 8))) - 1024);

	std::memcpy(&pre_uart_rx_data_, &tmp, sizeof(Dr16RecviedRawData));

	// 归一化
	float right_x = static_cast<float>(tmp.channel0) / kRockerNum;
	float right_y = static_cast<float>(tmp.channel1) / kRockerNum;
	float left_x = static_cast<float>(tmp.channel2) / kRockerNum;
	float left_y = static_cast<float>(tmp.channel3) / kRockerNum;

	right_x = std::clamp(right_x, -0.99f, 0.99f);
	right_y = std::clamp(right_y, -0.99f, 0.99f);
	left_x = std::clamp(left_x, -0.99f, 0.99f);
	left_y = std::clamp(left_y, -0.99f, 0.99f);

	// 鼠标处理
	const float mouse_x = static_cast<float>(tmp.mouse.x) / 32767.0f * kMouseSensitivityX;
	mouse_y = static_cast<float>(tmp.mouse.y) / 32767.0f * kMouseSensitivityY;
	mouse_y = std::clamp(mouse_y, -0.5f, 0.5f);

	const uint8_t yaw_u8 = clamp_to_u8((right_x + mouse_x) * 255.0f);
	const uint8_t pitch_u8 = clamp_to_u8((right_y + mouse_y) * 255.0f);

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

	const float wheel_norm = std::clamp(static_cast<float>(tmp.pulley_wheel) / kRockerNum, -1.0f, 1.0f);
	const uint8_t wheel_u8 = clamp_to_u8((wheel_norm + 1.0f) * 127.5f);

	const uint8_t stick_x_u8 = clamp_to_u8((left_x + 1.0f) * 127.5f);
	const uint8_t stick_y_u8 = clamp_to_u8((left_y + 1.0f) * 127.5f);
	const uint8_t out_x_u8 = (target_speed_x == kTargetCenter) ? stick_x_u8 : clamp_to_u8(target_speed_x);
	const uint8_t out_y_u8 = (target_speed_y == kTargetCenter) ? stick_y_u8 : clamp_to_u8(target_speed_y);

	// 拨杆映射到小陀螺：DOWN(1)->None, MID(3)->CW, UP(2)->CCW；SHIFT 强制 CW
	orb::ChassisSpin spin_from_sw = orb::ChassisSpin::ChassisSpinNone;
	switch (tmp.switch2) {
		case 1: spin_from_sw = orb::ChassisSpin::ChassisSpinNone; break;
		case 3: spin_from_sw = orb::ChassisSpin::ChassisSpinCW; break;
		case 2: spin_from_sw = orb::ChassisSpin::ChassisSpinCCW; break;
		default: spin_from_sw = orb::ChassisSpin::ChassisSpinNone; break;
	}

	orb::RcControl info{};
	info.yaw_angle = yaw_u8;
	info.pitch_angle = pitch_u8;
	info.chassis_speed_x = out_x_u8;
	info.chassis_speed_y = out_y_u8;
	info.chassis_rotation = wheel_u8;
	info.chassis_spin = (tmp.keyboard.bit.SHIFT != 0) ? orb::ChassisSpin::ChassisSpinCW : spin_from_sw;
	info.super_cap = (tmp.keyboard.bit.E != 0) ? orb::SuperCap::SuperCapDischarge
	                                           : orb::SuperCap::SuperCapCharge;
	info.auto_aim = (tmp.mouse.r != 0) ? orb::AutoAim::AutoAimOn : orb::AutoAim::AutoAimOff;
	info.gimbal_set_zero = (tmp.keyboard.bit.B != 0) ? orb::GimbalSetZero::GimbalSetZeroOn
	                                                 : orb::GimbalSetZero::GimbalSetZeroOff;
	info.friction_wheel = (tmp.keyboard.bit.F != 0) ? orb::FrictionWheel::FrictionWheelOn
	                                                : orb::FrictionWheel::FrictionWheelOff;
	info.shoot = (tmp.mouse.l != 0) ? orb::Shoot::ShootOn : orb::Shoot::ShootOff;
	info.eject = (tmp.keyboard.bit.R != 0) ? orb::Eject::EjectOn : orb::Eject::EjectOff;

	orb::rc_control.publish(info);
}

