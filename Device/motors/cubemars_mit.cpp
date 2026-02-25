#include "cubemars_mit.hpp"

#include "motor_registry.hpp"

#include "cmsis_os2.h"

#include "utils/alg_constrain.h"
#include "utils/alg_quantize.hpp"

extern "C" {
#include "FreeRTOS.h" // NOLINT(misc-include-cleaner)
#include "task.h"
}

static_assert(configASSERT_DEFINED == 1, "configASSERT_DEFINED expected");

#include "../../communication_topic/actuator_cmd_topics.hpp"
#include "../../communication_topic/actuator_state_topics.hpp"

#include <cstring>

namespace actuator::drivers {

namespace {
using alg::float_constrain;
using alg::float_to_uint;
using alg::uint_to_float;

static constexpr size_t kMaxMotors = 8;

// CubeMars motor registry: key=(bus + can_tx_id(tx) + can_rx_id(rx))
static MotorRegistry<CubemarsMitMin, kMaxMotors> s_registry{};

static StaticTask_t s_task_tcb;
static StackType_t s_task_stack[512];
static osThreadId_t s_task_thread = nullptr;

static void cubemars_task(void*)
{
    // 目标帧：angle/omega/torque + kp/kd
    RingSub<orb::CubemarsTargetCmd, 16> target_sub{orb::cubemars_target_cmd};
    // 管理帧：enter/exit/save_zero
    RingSub<orb::CubemarsAdminCmd, 8> admin_sub{orb::cubemars_admin_cmd};

    for (;;) {
        orb::CubemarsTargetCmd t{};
        while (target_sub.copy(t)) {
            CubemarsMitMin* m = s_registry.FindByBusRx(t.bus, static_cast<uint16_t>(t.can_rx_id));
            if (!m) {
                continue;
            }
            m->SetTarget(t.angle, t.omega, t.torque);
            m->PublishMitTx(t.kp, t.kd);
        }

        orb::CubemarsAdminCmd c{};
        while (admin_sub.copy(c)) {
            CubemarsMitMin* m = s_registry.FindByBusRx(c.bus, static_cast<uint16_t>(c.can_rx_id));
            if (!m) {
                continue;
            }
            switch (c.op) {
            case orb::CubemarsAdminOp::Enter:
                m->Enter();
                break;
            case orb::CubemarsAdminOp::Exit:
                m->Exit();
                break;
            case orb::CubemarsAdminOp::SaveZero:
                m->SaveZero();
                break;
            default:
                break;
            }
        }

        osDelay(1);
    }
}
} // namespace

void CubemarsMitMin::Init(BspCanHandle can, const Config& cfg)
{
    can_ = can;
    cfg_ = cfg;
    joined_runtime_ = false;
}

void CubemarsMitMin::JoinRuntime()
{
    configASSERT(can_ != nullptr);
    configASSERT(joined_runtime_ == false);
    if (joined_runtime_) {
        return;
    }

    const MotorKey key{cfg_.bus, cfg_.can_tx_id, static_cast<uint16_t>(cfg_.can_rx_id)};
    const bool stored = s_registry.RegisterOrReplace(key, this);
    configASSERT(stored);

    if (!s_task_thread) {
        static const osThreadAttr_t attr = {
            .name = "cubemars",
            .cb_mem = &s_task_tcb,
            .cb_size = sizeof(s_task_tcb),
            .stack_mem = s_task_stack,
            .stack_size = sizeof(s_task_stack),
            .priority = (osPriority_t)osPriorityAboveNormal,
        };
        s_task_thread = osThreadNew(cubemars_task, nullptr, &attr);
        configASSERT(s_task_thread != nullptr);
    }

    joined_runtime_ = true;
}

void CubemarsMitMin::CanRxCpltCallback(const BspCanFrame* frame)
{
    if (!frame) {
        return;
    }
    if (frame->id_type != BSP_CAN_ID_STD || frame->frame_type != BSP_CAN_FRAME_DATA || frame->len < 6u) {
        return;
    }

    const uint8_t* data = frame->data;

    // CubeMars v1 feedback frame (similar to MIT reply):
    // byte0: id
    // byte1-2: position (16)
    // byte3-4: velocity (12)
    // byte4-5: torque (12)
    const uint8_t id = data[0];
    if ((id & 0x0F) != (cfg_.can_rx_id & 0x0F)) {
        return;
    }

    const uint16_t p_u16 = (static_cast<uint16_t>(data[1]) << 8) | data[2];
    const uint16_t v_u12 = (static_cast<uint16_t>(data[3]) << 4) | (data[4] >> 4);
    const uint16_t t_u12 = (static_cast<uint16_t>(data[4] & 0x0F) << 8) | data[5];

    const float raw_angle = uint_to_float(p_u16, -cfg_.angle_max, cfg_.angle_max, 16);
    now_angle_ = raw_angle;
    now_omega_ = uint_to_float(v_u12, -cfg_.omega_max, cfg_.omega_max, 12);
    now_torque_ = uint_to_float(t_u12, -cfg_.torque_max, cfg_.torque_max, 12);

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

    orb::CubemarsFeedback fb{};
    fb.bus = cfg_.bus;
    fb.can_rx_id = (cfg_.can_rx_id & 0x0F);
    fb.angle_rad = now_angle_;
    fb.total_angle_rad = now_total_angle_;
    fb.omega_rad_s = now_omega_;
    fb.torque_nm = now_torque_;

    const int idx = orb::cubemars_feedback_index(fb.bus, fb.can_rx_id);
    orb::cubemars_feedback[idx].publish(fb);
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

    const uint16_t p_u16 = static_cast<uint16_t>(float_to_uint(p, -pmax, pmax, 16));
    const uint16_t v_u12 = static_cast<uint16_t>(float_to_uint(v, -vmax, vmax, 12));
    const uint16_t kp_u12 = static_cast<uint16_t>(float_to_uint(kp, 0, kpmax, 12));
    const uint16_t kd_u12 = static_cast<uint16_t>(float_to_uint(kd, 0, kdmax, 12));
    const uint16_t t_u12 = static_cast<uint16_t>(float_to_uint(t, -tmax, tmax, 12));

    out[0] = static_cast<uint8_t>((p_u16 >> 8) & 0xFF);
    out[1] = static_cast<uint8_t>(p_u16 & 0xFF);
    out[2] = static_cast<uint8_t>((v_u12 >> 4) & 0xFF);
    out[3] = static_cast<uint8_t>(((v_u12 & 0x0F) << 4) | ((kp_u12 >> 8) & 0x0F));
    out[4] = static_cast<uint8_t>(kp_u12 & 0xFF);
    out[5] = static_cast<uint8_t>((kd_u12 >> 4) & 0xFF);
    out[6] = static_cast<uint8_t>(((kd_u12 & 0x0F) << 4) | ((t_u12 >> 8) & 0x0F));
    out[7] = static_cast<uint8_t>(t_u12 & 0xFF);
}

void CubemarsMitMin::PublishFrame(uint16_t std_id, const uint8_t data[8], uint8_t len)
{
    (void)can_;
    orb::CanTxFrame f{};
    f.bus = cfg_.bus;
    f.id = std_id;
    f.id_type = orb::CanIdType::Std;
    f.frame_type = orb::CanFrameType::Data;
    f.is_fd = false;
    f.brs = false;
    f.len = len;
    std::memset(f.data, 0, sizeof(f.data));
    if (len > 0) {
        const uint8_t n = (len <= 8) ? len : 8;
        std::memcpy(f.data, data, n);
    }
    orb::can_tx.publish(f);
}

void CubemarsMitMin::PublishMitTx(float kp, float kd)
{
    uint8_t data[8];
    PackMit(ctrl_angle_, ctrl_omega_, kp, kd, ctrl_torque_, data,
            cfg_.angle_max, cfg_.omega_max, 500.0f, 5.0f, cfg_.torque_max);
    PublishFrame(cfg_.can_tx_id, data, 8);
}

void CubemarsMitMin::PublishAdminTail(uint8_t tail)
{
    uint8_t data[8];
    for (int i = 0; i < 7; ++i) {
        data[i] = 0xFF;
    }
    data[7] = tail;
    PublishFrame(cfg_.can_tx_id, data, 8);
}

void CubemarsMitMin::Enter() { PublishAdminTail(0xFC); }
void CubemarsMitMin::Exit() { PublishAdminTail(0xFD); }
void CubemarsMitMin::SaveZero() { PublishAdminTail(0xFE); }

} // namespace actuator::drivers

namespace actuator::instances {

actuator::drivers::CubemarsMitMin motor_z_axis_right{};
actuator::drivers::CubemarsMitMin motor_z_axis_left{};

} // namespace actuator::instances
