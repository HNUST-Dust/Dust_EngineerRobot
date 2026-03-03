#include "cubemars_mit.hpp"

#include "utils/alg_constrain.h"
#include "utils/alg_quantize.hpp"

#include "bsp_can_port.h"
#include "bsp_dwt.h"

#include <cstring>

namespace actuator::drivers {

namespace {
using alg::float_constrain;

static inline float wrap_delta(float delta, float angle_max)
{
    const float wrap = 2.0f * angle_max;
    if (delta > angle_max) {
        delta -= wrap;
    } else if (delta < -angle_max) {
        delta += wrap;
    }
    return delta;
}

} // namespace

void CubemarsMitMin::Init(BspCanHandle can, const Config& cfg)
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

void CubemarsMitMin::CanRxCpltCallback(const BspCanFrame* frame)
{
    if (!frame) {
        return;
    }
    // if (frame->id_type != BSP_CAN_ID_STD || frame->frame_type != BSP_CAN_FRAME_DATA || frame->len < 8u) {
    //     return;
    // }
    if (frame->id != cfg_.rx_std_id) {
        return;
    }

    // feedback format is MIT-like: [id][p16][v12][t12] ...
    const uint8_t* d = frame->data;

    // NOTE: old code used byte0 as id, but actual mapping depends on motor firmware.
    // Keep minimal check: accept any id in byte0.
    const uint16_t angle_u16 = (static_cast<uint16_t>(d[1]) << 8) | d[2];
    const uint16_t omega_u12 = (static_cast<uint16_t>(d[3]) << 4) | (d[4] >> 4);
    const uint16_t torque_u12 = (static_cast<uint16_t>(d[4] & 0x0F) << 8) | d[5];

    const float raw_angle = alg::uint_to_float(angle_u16, -cfg_.angle_max, cfg_.angle_max, 16);
    now_angle_ = raw_angle;
    now_omega_ = alg::uint_to_float(omega_u12, -cfg_.omega_max, cfg_.omega_max, 12);
    now_torque_ = alg::uint_to_float(torque_u12, -cfg_.torque_max, cfg_.torque_max, 12);

    if (!raw_angle_inited_) {
        raw_angle_inited_ = true;
        last_raw_angle_ = raw_angle;
        now_total_angle_ = raw_angle;
    } else {
        const float delta = wrap_delta(raw_angle - last_raw_angle_, cfg_.angle_max);
        now_total_angle_ += delta;
        last_raw_angle_ = raw_angle;
    }
}

void CubemarsMitMin::SetTarget(float angle_rad, float omega_rad_s, float torque_nm)
{
    ctrl_angle_ = float_constrain(angle_rad, -cfg_.angle_max, cfg_.angle_max);
    ctrl_omega_ = float_constrain(omega_rad_s, -cfg_.omega_max, cfg_.omega_max);
    ctrl_torque_ = float_constrain(torque_nm, -cfg_.torque_max, cfg_.torque_max);
}

void CubemarsMitMin::PackMit(float p, float v, float kp, float kd, float t, uint8_t out[8],
                             float pmax, float vmax, float kpmax, float kdmax, float tmax)
{
    std::memset(out, 0, 8);

    const uint16_t p_u16 = static_cast<uint16_t>(alg::float_to_uint(p, -pmax, pmax, 16));
    const uint16_t v_u12 = static_cast<uint16_t>(alg::float_to_uint(v, -vmax, vmax, 12));
    const uint16_t kp_u12 = static_cast<uint16_t>(alg::float_to_uint(kp, 0.0f, kpmax, 12));
    const uint16_t kd_u12 = static_cast<uint16_t>(alg::float_to_uint(kd, 0.0f, kdmax, 12));
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

BspCanFrame CubemarsMitMin::MakeMitFrame(uint16_t tx_std_id,
                                        float p, float v, float kp, float kd, float t,
                                        float pmax, float vmax, float kpmax, float kdmax, float tmax)
{
    BspCanFrame out{};
    out.id = tx_std_id;
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

BspCanFrame CubemarsMitMin::MakeAdminFrame(uint16_t tx_std_id, uint8_t tail)
{
    BspCanFrame out{};
    out.id = tx_std_id;
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

void CubemarsMitMin::PublishFrame(const BspCanFrame& f)
{
    if (!can_) {
        return;
    }
    bsp_can_send(can_, &f);
}

void CubemarsMitMin::PublishAdminTail(uint8_t tail)
{
    PublishFrame(MakeAdminFrame(cfg_.tx_std_id, tail));
}

void CubemarsMitMin::PublishMitTx(float kp, float kd)
{
    const float kp_c = float_constrain(kp, 0.0f, cfg_.kp_max);
    const float kd_c = float_constrain(kd, 0.0f, cfg_.kd_max);

    PublishFrame(MakeMitFrame(cfg_.tx_std_id,
                             ctrl_angle_, ctrl_omega_, kp_c, kd_c, ctrl_torque_,
                             cfg_.angle_max, cfg_.omega_max, cfg_.kp_max, cfg_.kd_max, cfg_.torque_max));

    // required behavior: send Enter after every control frame
    if (cfg_.enter_after_control) {
        Enter();
    }
}

void CubemarsMitMin::Enter() { PublishAdminTail(0xFC); }
void CubemarsMitMin::Exit() { PublishAdminTail(0xFD); }
void CubemarsMitMin::SaveZero() { PublishAdminTail(0xFE); }

void CubemarsMitMin::BringUpDefault()
{
    static constexpr float kDelaySaveZeroS = 1.0f;
    static constexpr float kDelayEnterS = 1.0f;

    SaveZero();
    dwt_delay(kDelaySaveZeroS);
    Enter();
    dwt_delay(kDelayEnterS);
}

} // namespace actuator::drivers
