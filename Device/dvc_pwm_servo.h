#ifndef DVC_PWM_SERVO_H
#define DVC_PWM_SERVO_H

#include "stm32h7xx_hal.h"

class DvcPwmServo {
public:
	// htim: 已配置为 PWM 输出的定时器句柄
	// channel: TIM 通道，例如 TIM_CHANNEL_1
	// min_pulse_ms / max_pulse_ms: 舵机控制脉冲范围（毫秒），默认 0.5ms - 2.5ms
	// full_angle_deg: 舵机的最大行程（度），例如 180/270/360
	// pwm_period_ms: PWM 周期，默认 20ms（50Hz），确保定时器 ARR 与此匹配
	DvcPwmServo(TIM_HandleTypeDef* htim, uint32_t channel,
				float min_pulse_ms = 0.5f, float max_pulse_ms = 2.5f,
				float full_angle_deg = 180.0f, float pwm_period_ms = 20.0f);

	// 启动 PWM（会调用 HAL_TIM_PWM_Start）
	HAL_StatusTypeDef begin();

	// 设置角度（度），0..full_angle_deg 范围内，超过会被限制
	void writeAngle(float angle_deg);

	// 直接设置脉冲宽度（毫秒），会被限制在 min/max 之间
	void writeMicroseconds(float pulse_ms);

	// 读取上次设置的角度
	float readAngle() const { return m_last_angle_deg; }

	float minPulseMs() const { return m_min_pulse_ms; }
	float maxPulseMs() const { return m_max_pulse_ms; }
	float fullAngle() const { return m_full_angle_deg; }
	float pwmPeriodMs() const { return m_pwm_period_ms; }

private:
	TIM_HandleTypeDef* m_htim;
	uint32_t m_channel;
	float m_min_pulse_ms;
	float m_max_pulse_ms;
	float m_full_angle_deg;
	float m_pwm_period_ms;
	float m_last_angle_deg;

	// 将毫秒脉冲宽度映射到定时器的 CCR（计数值）
	uint32_t pulseMsToCCR(float pulse_ms) const;
};

#endif // DVC_PWM_SERVO_H

