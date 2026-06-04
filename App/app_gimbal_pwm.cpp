#include "app_gimbal_pwm.h"
#include <algorithm>
#include <cstring>

// CMSIS-RTOS v2
#include "cmsis_os2.h"

struct PwmGimbal::Impl {
	std::vector<DvcPwmServo*> servos;
	std::vector<float> targets;
	osThreadId_t thread_id = nullptr;
	osMutexId_t mutex = nullptr;
	bool running = false;
	float update_hz = 50.0f;

	static void threadFunc(void* arg) {
		Impl* self = static_cast<Impl*>(arg);
		const uint32_t period_ms = (self->update_hz > 0.0f) ? (uint32_t)(1000.0f / self->update_hz) : 20U;
		while (true) {
			// snapshot under mutex
			std::vector<DvcPwmServo*> local_servos;
			std::vector<float> local_targets;
			osMutexAcquire(self->mutex, osWaitForever);
			bool run = self->running;
			if (!run) {
				osMutexRelease(self->mutex);
				break;
			}
			local_servos = self->servos;
			local_targets = self->targets;
			osMutexRelease(self->mutex);

			// write targets
			for (size_t i = 0; i < local_servos.size(); ++i) {
				DvcPwmServo* s = local_servos[i];
				if (s) s->writeAngle(local_targets[i]);
			}

			osDelay(period_ms);
		}
		// thread exits
	}
};

PwmGimbal::PwmGimbal(const std::vector<DvcPwmServo*>& servos, float update_hz)
	: m_servos(servos), m_update_hz(update_hz), m_impl(new Impl()) {
	m_impl->servos = servos;
	m_impl->targets.assign(servos.size(), 0.0f);
	m_impl->update_hz = update_hz;
	// create mutex
	m_impl->mutex = osMutexNew(nullptr);
}

PwmGimbal::~PwmGimbal() {
	stop();
	if (m_impl->mutex) osMutexDelete(m_impl->mutex);
	delete m_impl;
	m_impl = nullptr;
}

void PwmGimbal::start() {
	osMutexAcquire(m_impl->mutex, osWaitForever);
	if (m_impl->running) {
		osMutexRelease(m_impl->mutex);
		return;
	}
	// 启动各个舵机的 PWM 输出
	for (DvcPwmServo* s : m_impl->servos) {
		if (s) s->begin();
	}
	m_impl->running = true;
	osThreadAttr_t attr = {0};
	attr.name = "PwmGimbal";
	attr.attr_bits = 0U;
	attr.cb_mem = nullptr;
	attr.cb_size = 0U;
	attr.stack_mem = nullptr;
	attr.stack_size = 1024U; // 可按需调整
	m_impl->thread_id = osThreadNew(Impl::threadFunc, m_impl, &attr);
	osMutexRelease(m_impl->mutex);
}

void PwmGimbal::stop() {
	osMutexAcquire(m_impl->mutex, osWaitForever);
	if (!m_impl->running) {
		osMutexRelease(m_impl->mutex);
		return;
	}
	m_impl->running = false;
	osMutexRelease(m_impl->mutex);

	// 等待线程退出
	if (m_impl->thread_id) {
		// 尝试 join（若实现可用），否则 osThreadJoin 会阻塞直到线程结束
		osThreadJoin(m_impl->thread_id);
		m_impl->thread_id = nullptr;
	}
}

void PwmGimbal::setTargetAngle(size_t idx, float angle_deg) {
	osMutexAcquire(m_impl->mutex, osWaitForever);
	if (idx < m_impl->targets.size()) m_impl->targets[idx] = angle_deg;
	osMutexRelease(m_impl->mutex);
}

void PwmGimbal::setTargetAngles(const std::vector<float>& angles_deg) {
	osMutexAcquire(m_impl->mutex, osWaitForever);
	size_t n = std::min(angles_deg.size(), m_impl->targets.size());
	for (size_t i = 0; i < n; ++i) m_impl->targets[i] = angles_deg[i];
	osMutexRelease(m_impl->mutex);
}

std::vector<float> PwmGimbal::getTargetAngles() const {
	std::vector<float> out;
	osMutexAcquire(m_impl->mutex, osWaitForever);
	out = m_impl->targets;
	osMutexRelease(m_impl->mutex);
	return out;
}

