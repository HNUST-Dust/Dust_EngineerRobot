#include "dvc_vt03.h"
#include "../communication_topic/uart_topics.hpp"
#include "FreeRTOS.h"
#include "task.h"
#include <cstdint>
#include <cstring>
#include "../communication_topic/rc_control_topics.hpp"

void VT03::Init(BspUartHandle uart, orb::UartPort vt03_port)
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
    vt03_port_ = vt03_port;

    started_ = true;
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

        float wheel = (tmp->wheel - kRockerOffset) / kRockerNum;

        int mode_sw = 1;
        switch (tmp->mode_switch) {
            case 0: mode_sw = 0; break;
            case 1: mode_sw = 1; break;
            case 2: mode_sw = 2; break;
            default: mode_sw = 1; break;
        }

        float mouse_x = tmp->mouse_x / 32768.0f * kMouseSensitivityX;
        mouse_y += tmp->mouse_y / 32768.0f * kMouseSensitivityY;
        if (mouse_y > 0.5f) mouse_y = 0.5f;
        else if (mouse_y < -0.5f) mouse_y = -0.5f;

        uint8_t mouse_left = (tmp->mouse_left_key) ? 1 : 0;
        uint8_t mouse_right = (tmp->mouse_right_key) ? 1 : 0;
        uint8_t mouse_middle = (tmp->mouse_middle_key) ? 1 : 0;

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
            // 都不按 / 同时按：缓停止回中位
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
            // 都不按 / 同时按：缓停止回中位
            if (target_speed_y < kTargetCenter) {
                target_speed_y += kStopStep;
                if (target_speed_y > kTargetCenter) target_speed_y = kTargetCenter;
            } else if (target_speed_y > kTargetCenter) {
                target_speed_y -= kStopStep;
                if (target_speed_y < kTargetCenter) target_speed_y = kTargetCenter;
            }
        }
        
        orb::RcControl info{};
        info.yaw_angle = (right_x + mouse_x) * 255;
        info.pitch_angle = (right_y + mouse_y) * 255;
        info.chassis_speed_x = left_x + target_speed_x;
        info.chassis_speed_y = left_y +target_speed_y;
        info.chassis_rotation = wheel * 255;
        info.chassis_spin = (tmp->keyboard.bit.SHIFT != 0)
                       ? orb::ChassisSpin::ChassisSpinCW
                       : static_cast<orb::ChassisSpin>(mode_sw);
        info.super_cap = (pause != 0) ? orb::SuperCap::SuperCapDischarge : orb::SuperCap::SuperCapCharge;
        info.auto_aim = (mouse_right != 0) ? orb::AutoAim::AutoAimOn : orb::AutoAim::AutoAimOff;
        info.gimbal_set_zero = (tmp->keyboard.bit.B != 0) ? orb::GimbalSetZero::GimbalSetZeroOn : orb::GimbalSetZero::GimbalSetZeroOff;
        info.friction_wheel = (tmp->keyboard.bit.F != 0) ? orb::FrictionWheel::FrictionWheelOn : orb::FrictionWheel::FrictionWheelOff;
        info.shoot = (mouse_left != 0) ? orb::Shoot::ShootOn : orb::Shoot::ShootOff;
        info.eject = (right_key != 0) ? orb::Eject::EjectOn : orb::Eject::EjectOff;

        orb::rc_control.publish(info);
    }
}