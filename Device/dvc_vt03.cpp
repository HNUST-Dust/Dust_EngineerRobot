#include "dvc_vt03.h"
#include "FreeRTOS.h"
#include "task.h"
#include <algorithm>
#include <cstdint>
#include <cstring>
#include "../communication_topic/rc_control_topics.hpp"

void VT03::Init(BspUartHandle uart)
{
    if (started_) {
        configASSERT(false);
        return;
    }
    configASSERT(uart != nullptr);
    if (uart == nullptr) {
        return;
    }

    uart_ = uart;
    started_ = false;

    // 保证 rc_control 上电初值为 0
    {
        orb::RcControl rc{};
        orb::rc_control.publish(rc);
    }
}

float mouse_y = 0.0f; // 全局变量，记录鼠标y轴状态
float target_speed_x = 127.0f;
float target_speed_y = 127.0f;
void VT03::RxCpltCallback(uint8_t* buffer, uint16_t length) 
{
    if (!buffer || length < sizeof(RecviedRawData)) return;

    RecviedRawData const* tmp = reinterpret_cast<RecviedRawData const*>(buffer);

    if (tmp->start_of_frame_1 == 0xA9 && tmp->start_of_frame_2 == 0x53) {
        float right_x = (tmp->channel_0 - kRockerOffset) / kRockerNum;
        if (right_x > 0.99f) right_x = 0.99f;
        else if (right_x < -0.99f) right_x = -0.99f;

        float right_y = (tmp->channel_1 - kRockerOffset) / kRockerNum;
        if (right_y > 0.99f) right_y = 0.99f;
        else if (right_y < -0.99f) right_y = -0.99f;

        float left_x = (tmp->channel_2 - kRockerOffset) / kRockerNum;
        float left_y = (tmp->channel_3 - kRockerOffset) / kRockerNum;

        uint8_t left_key = (tmp->fn_1) ? 1 : 0;
        uint8_t right_key = (tmp->fn_2) ? 1 : 0;
        uint8_t trigger = (tmp->trigger) ? 1 : 0;
        uint8_t pause = (tmp->pause) ? 1 : 0;
        (void)left_key;
        (void)right_key;
        (void)trigger;
        (void)pause;

        float wheel = (tmp->wheel - kRockerOffset) / kRockerNum;

        float mouse_x = tmp->mouse_x / 32768.0f * kMouseSensitivityX;
        mouse_y += tmp->mouse_y / 32768.0f * kMouseSensitivityY;
        if (mouse_y > 0.5f) mouse_y = 0.5f;
        else if (mouse_y < -0.5f) mouse_y = -0.5f;

        uint8_t mouse_left = (tmp->mouse_left_key) ? 1 : 0;
        uint8_t mouse_right = (tmp->mouse_right_key) ? 1 : 0;
        uint8_t mouse_middle = (tmp->mouse_middle_key) ? 1 : 0;
        (void)mouse_left;
        (void)mouse_right;
        (void)mouse_middle;

        std::memcpy(&pre_uart_rx_data_, tmp, sizeof(RecviedRawData));

        // 键盘控制速度：缓启动 + 松键缓停止（回到中位 127）
        constexpr float kTargetMin = 0.0f;
        constexpr float kTargetMax = 255.0f;
        constexpr float kTargetCenter = 127.0f;
        constexpr float kStartStep = 1.0f;
        constexpr float kStopStep = 1.0f;

        const bool w_pressed = (tmp->keyboard.bit.W != 0);
        const bool s_pressed = (tmp->keyboard.bit.S != 0);
        const bool a_pressed = (tmp->keyboard.bit.A != 0);
        const bool d_pressed = (tmp->keyboard.bit.D != 0);

        // X 方向：W/S
        if (w_pressed && !s_pressed) {
            if (target_speed_x < kTargetMax) {
                target_speed_x += kStartStep;
                if (target_speed_x > kTargetMax) target_speed_x = kTargetMax;
            }
        } else if (s_pressed && !w_pressed) {
            if (target_speed_x > kTargetMin) {
                target_speed_x -= kStartStep;
                if (target_speed_x < kTargetMin) target_speed_x = kTargetMin;
            }
        } else {
            if (target_speed_x < kTargetCenter) {
                target_speed_x += kStopStep;
                if (target_speed_x > kTargetCenter) target_speed_x = kTargetCenter;
            } else if (target_speed_x > kTargetCenter) {
                target_speed_x -= kStopStep;
                if (target_speed_x < kTargetCenter) target_speed_x = kTargetCenter;
            }
        }

        // Y 方向：A/D
        if (d_pressed && !a_pressed) {
            if (target_speed_y < kTargetMax) {
                target_speed_y += kStartStep;
                if (target_speed_y > kTargetMax) target_speed_y = kTargetMax;
            }
        } else if (a_pressed && !d_pressed) {
            if (target_speed_y > kTargetMin) {
                target_speed_y -= kStartStep;
                if (target_speed_y < kTargetMin) target_speed_y = kTargetMin;
            }
        } else {
            if (target_speed_y < kTargetCenter) {
                target_speed_y += kStopStep;
                if (target_speed_y > kTargetCenter) target_speed_y = kTargetCenter;
            } else if (target_speed_y > kTargetCenter) {
                target_speed_y -= kStopStep;
                if (target_speed_y < kTargetCenter) target_speed_y = kTargetCenter;
            }
        }

        const float left_stick_x_norm = std::clamp((target_speed_y - kTargetCenter) / 127.0f, -1.0f, 1.0f);
        const float left_stick_y_norm = std::clamp((target_speed_x - kTargetCenter) / 127.0f, -1.0f, 1.0f);

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
        info.chassis_x_speed = left_stick_x_norm;
        info.chassis_y_speed = -left_stick_y_norm;
        info.chassis_rotation_speed = std::clamp(wheel, -1.0f, 1.0f);

        // arm
        info.arm_claw_angle = -std::clamp(right_x + mouse_x, -1.0f, 1.0f);

        // wrist: 积累
        wrist_yaw_acc += std::clamp(wheel, -1.0f, 1.0f) * kWristAccStep;
        wrist_pitch_acc += std::clamp(right_y + mouse_y, -1.0f, 1.0f) * kWristAccStep;
        info.arm_wrist_yaw_angle = wrist_yaw_acc;
        info.arm_wrist_pitch_angle = wrist_pitch_acc;

        // elbow: 积累
        elbow_pitch_acc += left_stick_y_norm * kElbowAccStep;
        elbow_yaw_acc += left_stick_x_norm * kElbowAccStep;
        info.arm_elbow_pitch_angle = elbow_pitch_acc;
        info.arm_elbow_yaw_angle = elbow_yaw_acc;

        // gantry: 积累
        gantry_x_acc += left_stick_y_norm * kGantryAccStep;
        gantry_y_acc += left_stick_x_norm * kGantryAccStep;
        gantry_z_acc += std::clamp(right_y + mouse_y, -1.0f, 1.0f) * kGantryAccStep;
        info.gantry_x_axis_distance = gantry_x_acc;
        info.gantry_y_axis_distance = gantry_y_acc;
        info.gantry_z_axis_distance = gantry_z_acc;

        // gimbal
        info.gimbal_pitch_angle = 0.0f;

        orb::rc_control.publish(info);

    }
}