#include "dm_mit.hpp"

#include "cmsis_os2.h"
#include "utils/alg_constrain.h"
#include "utils/alg_quantize.hpp"

#include "bsp_dwt.h"
#include "bsp_can_port.h"

#include <cstring>

namespace actuator::drivers {

namespace {
using alg::float_constrain;
} // namespace

void DmMitMin::Init(BspCanHandle can, const Config& cfg)
{
    can_ = can;
    cfg_ = cfg;

    now_angle_ = 0.0f;
    now_total_angle_ = 0.0f;
    now_omega_ = 0.0f;
    now_torque_ = 0.0f;

    raw_angle_inited_ = false;
    last_raw_angle_ = 0.0f;

    ctrl_angle_ = 0.0f;
    ctrl_omega_ = 0.0f;
    ctrl_torque_ = 0.0f;
}

void DmMitMin::CanRxCpltCallback(const BspCanFrame* frame) {
    if (!frame) {
        return;
    }
    if (frame->id_type != BSP_CAN_ID_STD || frame->frame_type != BSP_CAN_FRAME_DATA || frame->len < 8u) {
        return;
    }

    const uint8_t* data = frame->data;

    // DM 普通反馈帧（8 bytes）：
    // byte0: [id(4) | status(4)]
    // byte1-2: angle (16)
    // byte3-4: omega (12)
    // byte4-5: torque (12)
    // byte6: mos temp
    // byte7: rotor temp
    const uint8_t id4 = (data[0] & 0x0F);
    if (id4 != (cfg_.can_rx_id & 0x0F)) {
        return;
    }

    const uint8_t status4 = (data[0] >> 4);
    now_status_ = (status4 & 0x0F);
    now_mos_temp_c_ = data[6];
    now_rotor_temp_c_ = data[7];

    const uint16_t angle_u16 = (static_cast<uint16_t>(data[1]) << 8) | data[2];
    const uint16_t omega_u12 = (static_cast<uint16_t>(data[3]) << 4) | (data[4] >> 4);
    const uint16_t torque_u12 = (static_cast<uint16_t>(data[4] & 0x0F) << 8) | data[5];

    const float raw_angle = alg::uint_to_float(angle_u16, -cfg_.angle_max, cfg_.angle_max, 16);
    now_angle_ = raw_angle;
    now_omega_ = alg::uint_to_float(omega_u12, -cfg_.omega_max, cfg_.omega_max, 12);
    now_torque_ = alg::uint_to_float(torque_u12, -cfg_.torque_max, cfg_.torque_max, 12);

    // 角度展开得到累计角：基于环绕范围 [-angle_max, angle_max]。
    if (!raw_angle_inited_) {
        raw_angle_inited_ = true;
        last_raw_angle_ = raw_angle;
        now_total_angle_ = raw_angle;
    } else {
        float delta = raw_angle - last_raw_angle_;
        const float wrap = 2.0f * cfg_.angle_max;
        if (delta > cfg_.angle_max) {
            delta -= wrap;
        } else if (delta < -cfg_.angle_max) {
            delta += wrap;
        }
        now_total_angle_ += delta;
        last_raw_angle_ = raw_angle;
    }
}

void DmMitMin::SetControl(float angle, float omega, float torque) {
    SetTarget(angle, omega, torque);
}

void DmMitMin::SetTarget(float angle_rad, float omega_rad_s, float torque_nm) {
    ctrl_angle_ = float_constrain(angle_rad, -cfg_.angle_max, cfg_.angle_max);
    ctrl_omega_ = float_constrain(omega_rad_s, -cfg_.omega_max, cfg_.omega_max);
    ctrl_torque_ = float_constrain(torque_nm, -cfg_.torque_max, cfg_.torque_max);
}

void DmMitMin::PackMit(float p, float v, float kp, float kd, float t, uint8_t out[8],
                       float pmax, float vmax, float kpmax, float kdmax, float tmax) {
    // MIT 协议打包：p(16) v(12) kp(12) kd(12) t(12)
    std::memset(out, 0, 8);

    const uint16_t p_u16 = static_cast<uint16_t>(alg::float_to_uint(p, -pmax, pmax, 16));
    const uint16_t v_u12 = static_cast<uint16_t>(alg::float_to_uint(v, -vmax, vmax, 12));
    const uint16_t kp_u12 = static_cast<uint16_t>(alg::float_to_uint(kp, 0, kpmax, 12));
    const uint16_t kd_u12 = static_cast<uint16_t>(alg::float_to_uint(kd, 0, kdmax, 12));
    const uint16_t t_u12 = static_cast<uint16_t>(alg::float_to_uint(t, -tmax, tmax, 12));

    out[0] = static_cast<uint8_t>((p_u16 >> 8) & 0xFF);
    out[1] = static_cast<uint8_t>(p_u16 & 0xFF);
    out[2] = static_cast<uint8_t>((v_u12 >> 4) & 0xFF);
    out[3] = static_cast<uint8_t>(((v_u12 & 0x0F) << 4) | ((kp_u12 >> 8) & 0x0F));
    out[4] = static_cast<uint8_t>(kp_u12 & 0xFF);
    out[5] = static_cast<uint8_t>((kd_u12 >> 4) & 0xFF);
    out[6] = static_cast<uint8_t>(((kd_u12 & 0x0F) << 4) | ((t_u12 >> 8) & 0x0F));
    out[7] = static_cast<uint8_t>(t_u12 & 0xFF);
}

BspCanFrame DmMitMin::MakeMitFrame(uint16_t base_std_id, uint8_t can_rx_id,
                                  float p, float v, float kp, float kd, float t,
                                  float pmax, float vmax, float kpmax, float kdmax, float tmax)
{
    BspCanFrame out{};
    out.id = static_cast<uint16_t>(base_std_id) | static_cast<uint16_t>(can_rx_id);
    out.len = 8;
    out.id_type = BSP_CAN_ID_STD;
    out.frame_type = BSP_CAN_FRAME_DATA;
    out.is_fd = false;
    out.brs = false;
    out.from_fifo1 = false;
    std::memset(out.data, 0, sizeof(out.data));
    PackMit(p, v, kp, kd, t, out.data, pmax, vmax, kpmax, kdmax, tmax);
    return out;
}

BspCanFrame DmMitMin::MakeAdminFrame(uint16_t base_std_id, uint8_t can_rx_id, uint8_t tail)
{
    BspCanFrame out{};
    out.id = static_cast<uint16_t>(base_std_id) | static_cast<uint16_t>(can_rx_id);
    out.len = 8;
    out.id_type = BSP_CAN_ID_STD;
    out.frame_type = BSP_CAN_FRAME_DATA;
    out.is_fd = false;
    out.brs = false;
    out.from_fifo1 = false;
    for (int i = 0; i < 7; ++i) {
        out.data[i] = 0xFF;
    }
    out.data[7] = tail;
    return out;
}

void DmMitMin::PublishFrame(uint16_t std_id, const uint8_t data[8], uint8_t len) {
    if (!can_) {
        return;
    }
    BspCanFrame f{};
    f.id = std_id;
    f.len = len;
    if (len > 0) {
        const uint8_t n = (len <= 8) ? len : 8;
        std::memcpy(f.data, data, n);
    }
    f.id_type = BSP_CAN_ID_STD;
    f.frame_type = BSP_CAN_FRAME_DATA;
    f.is_fd = false;
    f.brs = false;
    f.from_fifo1 = false;
    bsp_can_send(can_, &f);
}

void DmMitMin::PublishMitTx(float kp, float kd) {
    uint8_t data[8];
    PackMit(ctrl_angle_, ctrl_omega_, kp, kd, ctrl_torque_, data,
            cfg_.angle_max, cfg_.omega_max, 500.0f, 5.0f, cfg_.torque_max);

    PublishFrame(cfg_.can_rx_id, data, 8);
}

void DmMitMin::PublishAdminTail(uint8_t tail)
{
    uint8_t data[8];
    for (int i = 0; i < 7; ++i) {
        data[i] = 0xFF;
    }
    data[7] = tail;
    PublishFrame(cfg_.can_rx_id, data, 8);
}

void DmMitMin::Enter() { PublishAdminTail(0xFC); }
void DmMitMin::Exit() { PublishAdminTail(0xFD); }
void DmMitMin::ClearError() { PublishAdminTail(0xFB); }
void DmMitMin::SaveZero() { PublishAdminTail(0xFE); }

void DmMitMin::BringUpDefault()
{
    ClearError();
    osDelay(100);
    Enter();
    osDelay(1000);
}

} // namespace actuator::drivers
