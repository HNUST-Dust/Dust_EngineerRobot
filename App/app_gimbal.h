#ifndef APP_GIMBAL_H_
#define APP_GIMBAL_H_

#include "dvc_servo.h"
#include "cmsis_os2.h"
#include <cstdint>

class Gimbal {
public:
    Gimbal();
    ServoBus servo_bus_;
    void Init(UART_HandleTypeDef *huart);

    void SetYawAngle(float degrees, uint32_t time_ms = 500);
    void SetPitchAngle(float degrees, uint32_t time_ms = 500);
    void SetYawSpeed(int16_t speed);
    void SetPitchSpeed(int16_t speed);
    void StopYaw();
    void StopPitch();

    uint8_t ReadID(uint8_t query_id = 0xFE, uint32_t timeout_ms = 200);
    uint8_t GetYawID() const { return yaw_id_; }
    uint8_t GetPitchID() const { return pitch_id_; }

    // thread entry
    static void TaskEntry(void *param);
    void Task();

private:
    volatile uint8_t yaw_id_{1};
    volatile uint8_t pitch_id_{2};
    osThreadId_t thread_id_{nullptr};
};

#endif // APP_GIMBAL_H_
