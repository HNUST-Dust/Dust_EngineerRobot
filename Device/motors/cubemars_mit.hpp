#pragma once

#include <cstdint>

#include "bsp_can_port.h"
#include "can_topics.hpp"

namespace actuator::drivers {

// CubeMars MIT-like protocol minimal driver, refactor of legacy `MotorCubemars`.
// Design goal: unify the runtime + Topic model with `DmMitMin`.
class CubemarsMitMin final {
public:
    struct Config {
        orb::CanBus bus = orb::CanBus::CAN1;
        // motor low 4-bit id in reply frame.
        uint8_t can_rx_id = 0x01;
        // CAN std id used when sending command/admin frames.
        uint16_t can_tx_id = 0x01;

        float angle_max = 12.5f;
        float omega_max = 50.0f;
        float torque_max = 65.0f;
    };

    void Init(BspCanHandle can, const Config& cfg);
    void JoinRuntime();

    void CanRxCpltCallback(const BspCanFrame* frame);

    void SetTarget(float angle_rad, float omega_rad_s, float torque_nm);

    void PublishMitTx(float kp = 0.0f, float kd = 0.0f);

    void Enter();
    void Exit();
    void SaveZero();

    // feedback
    float now_angle_rad() const { return now_angle_; }
    float now_total_angle_rad() const { return now_total_angle_; }
    float now_omega_rad_s() const { return now_omega_; }
    float now_torque_nm() const { return now_torque_; }

    // target
    float target_angle_rad() const { return ctrl_angle_; }
    float target_omega_rad_s() const { return ctrl_omega_; }
    float target_torque_nm() const { return ctrl_torque_; }

    orb::CanBus bus() const { return cfg_.bus; }
    uint8_t can_rx_id() const { return cfg_.can_rx_id; }
    uint16_t can_tx_id() const { return cfg_.can_tx_id; }

private:
    Config cfg_{};
    BspCanHandle can_ = nullptr;
    bool joined_runtime_ = false;

    // feedback state
    float now_angle_ = 0.0f;
    float now_total_angle_ = 0.0f;
    float now_omega_ = 0.0f;
    float now_torque_ = 0.0f;

    bool raw_angle_inited_ = false;
    float last_raw_angle_ = 0.0f;

    // target state
    float ctrl_angle_ = 0.0f;
    float ctrl_omega_ = 0.0f;
    float ctrl_torque_ = 0.0f;

    static void PackMit(float p, float v, float kp, float kd, float t, uint8_t out[8],
                        float pmax, float vmax, float kpmax, float kdmax, float tmax);

    void PublishFrame(uint16_t std_id, const uint8_t data[8], uint8_t len);
    void PublishAdminTail(uint8_t tail);
};

} // namespace actuator::drivers

namespace actuator::instances {

extern actuator::drivers::CubemarsMitMin motor_z_axis_right;
extern actuator::drivers::CubemarsMitMin motor_z_axis_left;

} // namespace actuator::instances
