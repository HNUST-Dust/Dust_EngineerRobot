#pragma once

#include <cstdint>

#include "bsp_can_port.h"

namespace actuator::drivers {

/**
 * @brief 达妙电机控制状态, 传统模式有效
 *
 */
enum Status
{
    MOTOR_DM_CONTROL_STATUS_DISABLE = 0x0,
    MOTOR_DM_CONTROL_STATUS_ENABLE,
    MOTOR_DM_CONTROL_STATUS_OVERVOLTAGE = 0x8,
    MOTOR_DM_CONTROL_STATUS_UNDERVOLTAGE,
    MOTOR_DM_CONTROL_STATUS_OVERCURRENT,
    MOTOR_DM_CONTROL_STATUS_MOS_OVERTEMPERATURE,
    MOTOR_DM_CONTROL_STATUS_ROTOR_OVERTEMPERATURE,
    MOTOR_DM_CONTROL_STATUS_LOSE_CONNECTION,
    MOTOR_DM_CONTROL_STATUS_MOS_OVERLOAD,
};

class DmMitMin final {
public:
    struct Config {
        uint8_t bus = 1; // 1->CAN1, 2->CAN2 (由上层自行约定)

        // 电机的 CAN ID，同时作为发送帧 std_id
        uint8_t can_rx_id = 0x01;

        // 主机 ID（用于过滤反馈帧 byte0 高4bit，通常不需要修改）
        uint8_t master_id = 0x01;

        float angle_max = 12.5f;
        float omega_max = 45.0f;
        float torque_max = 10.0f;
    };

    void Init(BspCanHandle can, const Config& cfg);

    // CAN RX complete callback entry (called from bus-level ID dispatch)
    void CanRxCpltCallback(const BspCanFrame* frame);

    // 设置目标（角度/角速度/力矩），纯协议层只存储，发送由 PublishMitTx 触发
    void SetTarget(float angle_rad, float omega_rad_s, float torque_nm);
    void SetControl(float angle, float omega, float torque);

    // 发送 MIT 控制帧（p/v/kp/kd/t）
    void PublishMitTx(float kp = 0.0f, float kd = 0.0f);

    // 管理帧（发送 0xFF...tail）
    void Enter();
    void Exit();
    void ClearError();
    void SaveZero();

    // 上电 bring-up 辅助（阻塞、best-effort）。内部用 DWT delay，可在 osKernelStart 前调用。
    void BringUpDefault();

    // 反馈状态（单位：rad / rad/s / N·m）
    float now_angle_rad() const { return now_angle_; }
    float now_total_angle_rad() const { return now_total_angle_; }
    float now_omega_rad_s() const { return now_omega_; }
    float now_torque_nm() const { return now_torque_; }

    // 反馈附加信息（来自反馈帧 byte0/status、byte6/byte7 温度）
    Status  now_status_enum() const { return now_status_; }
    uint8_t now_mos_temp_c() const { return now_mos_temp_c_; }
    uint8_t now_rotor_temp_c() const { return now_rotor_temp_c_; }

    // 是否处于使能状态
    bool is_enabled() const { return now_status_ == MOTOR_DM_CONTROL_STATUS_ENABLE; }
    // 是否处于错误状态（过压/欠压/过流/过温/过载等）
    bool is_error() const { return now_status_ >= MOTOR_DM_CONTROL_STATUS_OVERVOLTAGE; }

    // 目标状态（单位：rad / rad/s / N·m）
    float target_angle_rad() const { return ctrl_angle_; }
    float target_omega_rad_s() const { return ctrl_omega_; }
    float target_torque_nm() const { return ctrl_torque_; }

    uint8_t bus() const { return cfg_.bus; }
    uint16_t rx_id() const { return static_cast<uint16_t>(cfg_.can_rx_id); }
    uint16_t tx_id() const { return static_cast<uint16_t>(cfg_.can_rx_id); }

    // ---- 纯协议层工具函数：MIT 协议打包 ----
    static void PackMit(float p, float v, float kp, float kd, float t, uint8_t out[8],
                        float pmax, float vmax, float kpmax, float kdmax, float tmax);

    // 直接构造 MIT 控制帧（len=8, std_id = base_std_id | (can_rx_id&0x0F)）
    static BspCanFrame MakeMitFrame(uint16_t base_std_id, uint8_t can_rx_id,
                                    float p, float v, float kp, float kd, float t,
                                    float pmax, float vmax, float kpmax, float kdmax, float tmax);

    // 直接构造 MIT 管理帧（len=8, 0xFF*7 + tail）
    static BspCanFrame MakeAdminFrame(uint16_t base_std_id, uint8_t can_rx_id, uint8_t tail);

private:
    Config cfg_{};
    BspCanHandle can_ = nullptr;

    float now_angle_ = 0.0f;
    float now_total_angle_ = 0.0f;
    float now_omega_ = 0.0f;
    float now_torque_ = 0.0f;

    Status  now_status_     = MOTOR_DM_CONTROL_STATUS_DISABLE;
    uint8_t now_mos_temp_c_ = 0;
    uint8_t now_rotor_temp_c_ = 0;

    bool raw_angle_inited_ = false;
    float last_raw_angle_ = 0.0f;

    float ctrl_angle_ = 0.0f;
    float ctrl_omega_ = 0.0f;
    float ctrl_torque_ = 0.0f;

    void PublishFrame(uint16_t std_id, const uint8_t data[8], uint8_t len);
    void PublishAdminTail(uint8_t tail);
};
} // namespace actuator::drivers
