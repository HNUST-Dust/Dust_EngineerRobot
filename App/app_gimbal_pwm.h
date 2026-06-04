#ifndef APP_GIMBAL_PWM_H
#define APP_GIMBAL_PWM_H

#include <vector>
#include <memory>
#include "dvc_pwm_servo.h"
#include "cmsis_os2.h" // 使用 CMSIS-RTOS v2（FreeRTOS）API

class PwmGimbal {
public:
	// 构造：传入一组已创建的 DvcPwmServo*（类并不拥有 servo 对象所有权）
	explicit PwmGimbal(const std::vector<DvcPwmServo*>& servos, float update_hz = 50.0f);
	~PwmGimbal();

	// 启动/停止控制线程
	void start();
	void stop();

	// 设置单个舵机目标角度（deg）
	void setTargetAngle(size_t idx, float angle_deg);
	// 批量设置目标角度（向量长度须与 servos 数一致）
	void setTargetAngles(const std::vector<float>& angles_deg);

	// 读取当前目标角度
	std::vector<float> getTargetAngles() const;

private:
	std::vector<DvcPwmServo*> m_servos;
	float m_update_hz;

	// 线程与同步由 cpp 实现
	struct Impl;
	Impl* m_impl;
};

#endif // APP_GIMBAL_PWM_H

