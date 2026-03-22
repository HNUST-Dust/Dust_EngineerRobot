#include "app_gimbal.h"
#include "usart.h"
#include "debug_tools.h"
#include "FreeRTOS.h"
#include "cmsis_os2.h"

Gimbal::Gimbal() = default;

void Gimbal::Init(UART_HandleTypeDef *huart)
{
    servo_bus_.Init(huart);
    static const osThreadAttr_t kGimbalTaskAttr = {
        .name = "gimbal_task",
        .stack_size = 512,
        .priority = (osPriority_t) osPriorityAboveNormal
    };
    thread_id_ = osThreadNew(Gimbal::TaskEntry, this, &kGimbalTaskAttr);
}

void Gimbal::SetYawAngle(float degrees, uint32_t time_ms)
{
    servo_bus_.MoveToAngle(yaw_id_, degrees, time_ms);
}

void Gimbal::SetPitchAngle(float degrees, uint32_t time_ms)
{
    servo_bus_.MoveToAngle(pitch_id_, degrees, time_ms);
}

void Gimbal::SetYawSpeed(int16_t speed)
{
    servo_bus_.SetSpeed(yaw_id_, speed);
}

void Gimbal::SetPitchSpeed(int16_t speed)
{
    servo_bus_.SetSpeed(pitch_id_, speed);
}

void Gimbal::StopYaw()
{
    servo_bus_.Stop(yaw_id_);
}

void Gimbal::StopPitch()
{
    servo_bus_.Stop(pitch_id_);
}

uint8_t Gimbal::ReadID(uint8_t query_id, uint32_t timeout_ms)
{
    return servo_bus_.ReadID(query_id, timeout_ms);
}

void Gimbal::TaskEntry(void *param)
{
    Gimbal *self = static_cast<Gimbal *>(param);
    self->Task();
}

void Gimbal::Task()
{
    // 在任务启动时尝试通过广播读取单个舵机的 ID（适用于总线上仅接一台舵机的情况）
    // uint8_t found = servo_bus_.ReadID(0xFE, 300);
    // if (found != 0xFF) {
    //     yaw_id_ = found;
    //     // 如果发现当前 ID 为 1，则把它设置为 2
    // //     if (found == 1) {
            // if (servo_bus_.WriteID(1, 2)) {
            //     // 等待舵机应用新 ID
            //     osDelay(pdMS_TO_TICKS(100));
                uint8_t confirm = servo_bus_.ReadID(0xFE, 300);
                yaw_id_ = confirm;
            // }
    //     }
    // }

    // 任务运行后保持低频循环，如需扩展可在此加入状态机或消息队列处理
    for (;;) {
        osDelay(pdMS_TO_TICKS(1000));
    }
}
