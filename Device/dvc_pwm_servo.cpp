#include "dvc_pwm_servo.h"
#include <algorithm>

DvcPwmServo::DvcPwmServo(TIM_HandleTypeDef* htim, uint32_t channel,
						 float min_pulse_ms, float max_pulse_ms,
						 float full_angle_deg, float pwm_period_ms)
	: m_htim(htim), m_channel(channel), m_min_pulse_ms(min_pulse_ms),
	  m_max_pulse_ms(max_pulse_ms), m_full_angle_deg(full_angle_deg),
	  m_pwm_period_ms(pwm_period_ms), m_last_angle_deg(0.0f) {}

HAL_StatusTypeDef DvcPwmServo::begin() {
	if (m_htim == nullptr) return HAL_ERROR;
	return HAL_TIM_PWM_Start(m_htim, m_channel);
}

void DvcPwmServo::writeAngle(float angle_deg) {
	if (m_htim == nullptr) return;
	// 限制角度
	float a = std::max(0.0f, std::min(angle_deg, m_full_angle_deg));
	// 映射到脉冲宽度
	float frac = (m_full_angle_deg <= 0.0f) ? 0.0f : (a / m_full_angle_deg);
	float pulse = m_min_pulse_ms + frac * (m_max_pulse_ms - m_min_pulse_ms);
	writeMicroseconds(pulse);
	m_last_angle_deg = a;
}

void DvcPwmServo::writeMicroseconds(float pulse_ms) {
	if (m_htim == nullptr) return;
	// 限制脉冲
	float p = std::max(m_min_pulse_ms, std::min(pulse_ms, m_max_pulse_ms));
	uint32_t ccr = pulseMsToCCR(p);
	__HAL_TIM_SET_COMPARE(m_htim, m_channel, ccr);
}

uint32_t DvcPwmServo::pulseMsToCCR(float pulse_ms) const {
	// 计算 CCR 的比例： CCR = (pulse_ms / pwm_period_ms) * period_ticks
	// period_ticks = ARR + 1
	uint32_t period_ticks = 1;
	if (m_htim && m_htim->Instance) {
		period_ticks = (uint32_t)(m_htim->Instance->ARR) + 1U;
	}
	float ratio = (m_pwm_period_ms <= 0.0f) ? 0.0f : (pulse_ms / m_pwm_period_ms);
	if (ratio < 0.0f) ratio = 0.0f;
	if (ratio > 1.0f) ratio = 1.0f;
	uint32_t ccr = (uint32_t)(ratio * (float)period_ticks + 0.5f);
	if (ccr > period_ticks) ccr = period_ticks;
	return ccr;
}

