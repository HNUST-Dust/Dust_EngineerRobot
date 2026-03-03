#pragma once

#include <cstdint>

#include "bsp_can_port.h"

namespace actuator::drivers {

// Cubemars MIT-like protocol minimal driver (protocol layer only)
class CubemarsMitMin final {
public:
    struct Config {
        uint8_t bus = 1;

        // feedback frame std id (master id)
        uint16_t rx_std_id = 0x000;

        // command frame std id
        uint16_t tx_std_id = 0x001;

        float angle_max = 12.5f;
        float omega_max = 50.0f;
        float torque_max = 65.0f;

        // protocol constants
        float kp_max = 500.0f;
        float kd_max = 5.0f;

        // requirement: after each control frame, send Enter admin frame
        bool enter_after_control = true;
    };

    void Init(BspCanHandle can, const Config& cfg);

    void CanRxCpltCallback(const BspCanFrame* frame);

    // set target stored in protocol layer
    void SetTarget(float angle_rad, float omega_rad_s, float torque_nm);

    float now_angle_rad() const { return now_angle_; }
    float now_total_angle_rad() const { return now_total_angle_; }
    float now_omega_rad_s() const { return now_omega_; }
    float now_torque_nm() const { return now_torque_; }

    float target_angle_rad() const { return ctrl_angle_; }
    float target_omega_rad_s() const { return ctrl_omega_; }
    float target_torque_nm() const { return ctrl_torque_; }

    // send control frame (p/v/kp/kd/t)
    void PublishMitTx(float kp = 0.0f, float kd = 0.0f);

    // admin frames
    void Enter();
    void Exit();
    void SaveZero();

    void BringUpDefault();

    // ---- helpers ----
    static void PackMit(float p, float v, float kp, float kd, float t, uint8_t out[8],
                        float pmax, float vmax, float kpmax, float kdmax, float tmax);

    static BspCanFrame MakeMitFrame(uint16_t tx_std_id,
                                    float p, float v, float kp, float kd, float t,
                                    float pmax, float vmax, float kpmax, float kdmax, float tmax);

    static BspCanFrame MakeAdminFrame(uint16_t tx_std_id, uint8_t tail);

private:
    Config cfg_{};
    BspCanHandle can_ = nullptr;

    float now_angle_ = 0.0f;
    float now_total_angle_ = 0.0f;
    float now_omega_ = 0.0f;
    float now_torque_ = 0.0f;

    bool raw_angle_inited_ = false;
    float last_raw_angle_ = 0.0f;

    float ctrl_angle_ = 0.0f;
    float ctrl_omega_ = 0.0f;
    float ctrl_torque_ = 0.0f;

    void PublishFrame(const BspCanFrame& f);
    void PublishAdminTail(uint8_t tail);
};

} // namespace actuator::drivers
