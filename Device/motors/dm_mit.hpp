#pragma once

#include <cstdint>

#include "bsp_can_port.h"
#include "control/alg_pid.h"
#include "can_topics.hpp"

namespace actuator::drivers {

class DmMitMin final {
public:
    enum class CascadeAxis : uint8_t {
        Yaw = 0,
        Pitch,
        Unknown = 255,
    };

    // Cascade feedback source selection (runtime, per motor & per loop)
    enum class CascadeFeedback : uint8_t {
        Encoder = 0,
        ImuLocal,
        ImuMcu,
    };

    struct Config {
        orb::CanBus bus = orb::CanBus::CAN1;
        // 统一命名:rx_id（该电机的低 4bit ID）
        uint8_t can_rx_id = 0x01;
        uint8_t master_id = 0x01;
        // 统一命名:tx_id（MIT 协议发送 std_id 的基址，高位部分）
        uint16_t base_std_id = 0x00;

        float angle_max = 12.5f;
        float omega_max = 45.0f;
        float torque_max = 10.0f;

        // Control loop rate (per-motor)
        // - This rate is also the CAN command publish rate (control frequency == send frequency).
        // - 0: no limit (run every driver cycle)
        // - otherwise: run at most control_rate_hz times per second
        uint16_t control_rate_hz = 0;

        // Feedback source selection for cascade loops
        CascadeFeedback angle_feedback = CascadeFeedback::Encoder;
        CascadeFeedback omega_feedback = CascadeFeedback::Encoder;
    };

    void Init(BspCanHandle can, const Config& cfg);

    void JoinRuntime();
    void CanRxCpltCallback(const BspCanFrame* frame);

    void SetTarget(float angle_rad, float omega_rad_s, float torque_nm);
    void SetControl(float angle, float omega, float torque);

    void PublishMitTx(float kp = 0.0f, float kd = 0.0f);

    void ConfigureCascadePid(const alg::PidConfig& angle_cfg,
                             const alg::PidConfig& omega_cfg,
                             float angle_to_omega_sign,
                             float omega_feedback_sign);

    void ConfigureGimbalCascadePidDefaults(CascadeAxis axis, CascadeFeedback fb);

    CascadeAxis cascade_axis() const { return cascade_axis_; }
    CascadeFeedback angle_feedback() const { return cfg_.angle_feedback; }
    CascadeFeedback omega_feedback() const { return cfg_.omega_feedback; }

    enum class CascadeMode : uint8_t {
        Angle = 0,
        Omega,
    };

    float UpdateCascadeTorque(CascadeMode mode,
                             float target_angle_rad,
                             float target_omega_rad_s,
                             float omega_ff_rad_s,
                             float measured_angle_rad,
                             float measured_omega_rad_s,
                             float* out_omega_sp_rad_s = nullptr);

    float UpdateAngleToOmega(float target_angle_rad, float measured_angle_rad);
    float UpdateOmegaToTorque(float target_omega_rad_s, float measured_omega_rad_s);
    float UpdateCascadeTorqueAngleMode(float target_angle_rad,
                                       float measured_angle_rad,
                                       float measured_omega_rad_s);

    void BringUpDefault();

    void Enter();
    void Exit();
    void ClearError();
    void SaveZero();

    float now_angle() const { return now_angle_; }
    float now_omega() const { return now_omega_; }
    float now_torque() const { return now_torque_; }
    float now_angle_rad() const { return now_angle_; }
    float now_total_angle_rad() const { return now_total_angle_; }
    float now_omega_rad_s() const { return now_omega_; }
    float now_torque_nm() const { return now_torque_; }

    float target_angle_rad() const { return ctrl_angle_; }
    float target_omega_rad_s() const { return ctrl_omega_; }
    float target_torque_nm() const { return ctrl_torque_; }

    orb::CanBus bus() const { return cfg_.bus; }
    uint8_t can_rx_id() const { return cfg_.can_rx_id; }
    uint16_t rx_id() const { return static_cast<uint16_t>(cfg_.can_rx_id); }
    uint16_t tx_id() const { return cfg_.base_std_id; }

    uint32_t ctrl_period_ticks() const { return ctrl_period_ticks_; }

private:
    Config cfg_{};
    BspCanHandle can_ = nullptr;
    bool joined_runtime_ = false;

    // Control scheduling / rate limiting state
    uint32_t last_ctrl_tick_ = 0;
    uint32_t ctrl_period_ticks_ = 0;

    float now_angle_ = 0.0f;
    float now_total_angle_ = 0.0f;
    float now_omega_ = 0.0f;
    float now_torque_ = 0.0f;

    bool raw_angle_inited_ = false;
    float last_raw_angle_ = 0.0f;

    float ctrl_angle_ = 0.0f;
    float ctrl_omega_ = 0.0f;
    float ctrl_torque_ = 0.0f;

    bool cascade_pid_inited_ = false;
    float angle_to_omega_sign_ = 1.0f;
    float omega_feedback_sign_ = 1.0f;
    float last_torque_cmd_ = 0.0f;

    CascadeAxis cascade_axis_ = CascadeAxis::Unknown;

    alg::Pid pid_angle_{};
    alg::Pid pid_omega_{};

    static void PackMit(float p, float v, float kp, float kd, float t, uint8_t out[8],
                        float pmax, float vmax, float kpmax, float kdmax, float tmax);

    void PublishFrame(uint16_t std_id, const uint8_t data[8], uint8_t len);
    void PublishAdminTail(uint8_t tail);
};
} // namespace actuator::drivers

namespace actuator::instances {

extern actuator::drivers::DmMitMin motor_x_axis_right;
extern actuator::drivers::DmMitMin motor_x_axis_left;
extern actuator::drivers::DmMitMin elbow_joint_pitch;
extern actuator::drivers::DmMitMin elbow_joint_yaw;
extern actuator::drivers::DmMitMin claws;

} // namespace actuator::instances
