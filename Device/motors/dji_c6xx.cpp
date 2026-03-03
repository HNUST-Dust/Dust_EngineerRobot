#include "dji_c6xx.hpp"

#include "utils/alg_constrain.h"

#include <cstring>

#include "bsp_can_port.h"

namespace actuator::drivers {

namespace {
using alg::float_constrain;
} // namespace

void DjiC6xxMin::Init(BspCanHandle can, const Config& cfg)
{
    cfg_ = cfg;
    can_ = can;

    last_enc_ = 0;
    total_round_ = 0;
    now_angle_ = 0.0f;
    now_omega_out_ = 0.0f;
    now_current_ = 0.0f;
    temperature_ = 0.0f;

    target_current_ = 0.0f;
}

void DjiC6xxMin::CanRxCpltCallback(const BspCanFrame* frame) {
    if (!frame) {
        return;
    }
    if (frame->id_type != BSP_CAN_ID_STD || frame->frame_type != BSP_CAN_FRAME_DATA || frame->len < 8u) {
        return;
    }
    if (frame->id != cfg_.rx_std_id) {
        return;
    }

    const uint8_t* data = frame->data;
    const uint16_t enc = (static_cast<uint16_t>(data[0]) << 8) | static_cast<uint16_t>(data[1]);
    const int16_t omega_rpm =
        static_cast<int16_t>((static_cast<uint16_t>(data[2]) << 8) | static_cast<uint16_t>(data[3]));
    const int16_t current_raw =
        static_cast<int16_t>((static_cast<uint16_t>(data[4]) << 8) | static_cast<uint16_t>(data[5]));
    const uint8_t temp = data[6];

    // unwrap encoder
    if (last_enc_ != 0) {
        int32_t diff = static_cast<int32_t>(enc) - static_cast<int32_t>(last_enc_);
        if (diff > 4096) {
            total_round_ -= 1;
        } else if (diff < -4096) {
            total_round_ += 1;
        }
    }
    last_enc_ = enc;

    const int32_t total_enc = total_round_ * static_cast<int32_t>(cfg_.enc_per_round) + static_cast<int32_t>(enc);
    const float motor_angle = (static_cast<float>(total_enc) / static_cast<float>(cfg_.enc_per_round)) * k2pi;
    now_angle_ = (cfg_.gearbox_ratio != 0.0f) ? (motor_angle / cfg_.gearbox_ratio) : motor_angle;

    // omega: rpm -> rad/s (motor side) then / gearbox_ratio to output side
    const float omega_motor = (static_cast<float>(omega_rpm) * k2pi) / 60.0f;
    now_omega_out_ = (cfg_.gearbox_ratio != 0.0f) ? (omega_motor / cfg_.gearbox_ratio) : omega_motor;

    // current: raw -> A (C6xx/GM6020 commonly uses 16384->20A scaling)
    now_current_ = static_cast<float>(current_raw) * (20.0f / 16384.0f);
    temperature_ = static_cast<float>(temp);
}

void DjiC6xxMin::SetTargetCurrent(float current) {
    target_current_ = float_constrain(current, -cfg_.current_limit, cfg_.current_limit);
}

int16_t DjiC6xxMin::target_current_raw() const {
    const float a = float_constrain(target_current_, -cfg_.current_limit, cfg_.current_limit);
    return static_cast<int16_t>(a * (16384.0f / 20.0f));
}

void DjiC6xxMin::PackGroupCurrent(int16_t i0, int16_t i1, int16_t i2, int16_t i3, uint8_t out[8])
{
    if (!out) {
        return;
    }
    const uint16_t c0 = static_cast<uint16_t>(i0);
    const uint16_t c1 = static_cast<uint16_t>(i1);
    const uint16_t c2 = static_cast<uint16_t>(i2);
    const uint16_t c3 = static_cast<uint16_t>(i3);

    out[0] = static_cast<uint8_t>((c0 >> 8) & 0xFF);
    out[1] = static_cast<uint8_t>(c0 & 0xFF);
    out[2] = static_cast<uint8_t>((c1 >> 8) & 0xFF);
    out[3] = static_cast<uint8_t>(c1 & 0xFF);
    out[4] = static_cast<uint8_t>((c2 >> 8) & 0xFF);
    out[5] = static_cast<uint8_t>(c2 & 0xFF);
    out[6] = static_cast<uint8_t>((c3 >> 8) & 0xFF);
    out[7] = static_cast<uint8_t>(c3 & 0xFF);
}

BspCanFrame DjiC6xxMin::MakeGroupCurrentFrame(uint16_t tx_std_id,
                                             int16_t i0, int16_t i1, int16_t i2, int16_t i3)
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
    PackGroupCurrent(i0, i1, i2, i3, out.data);
    return out;
}

void DjiC6xxMin::SendGroup(DjiC6xxMin* m0, DjiC6xxMin* m1, DjiC6xxMin* m2, DjiC6xxMin* m3)
{
    // 取第一个非空电机的 can 句柄和 tx_std_id 发送
    const DjiC6xxMin* ref = m0 ? m0 : (m1 ? m1 : (m2 ? m2 : m3));
    if (!ref || !ref->can_) {
        return;
    }
    const int16_t i0 = m0 ? m0->target_current_raw() : int16_t{0};
    const int16_t i1 = m1 ? m1->target_current_raw() : int16_t{0};
    const int16_t i2 = m2 ? m2->target_current_raw() : int16_t{0};
    const int16_t i3 = m3 ? m3->target_current_raw() : int16_t{0};
    BspCanFrame f = MakeGroupCurrentFrame(ref->cfg_.tx_std_id, i0, i1, i2, i3);
    bsp_can_send(ref->can_, &f);
}

void DjiC6xxMin::SendGroupCurrent(BspCanHandle can, uint16_t tx_std_id,
                                  int16_t i0, int16_t i1, int16_t i2, int16_t i3)
{
    if (!can) {
        return;
    }
    BspCanFrame f = MakeGroupCurrentFrame(tx_std_id, i0, i1, i2, i3);
    bsp_can_send(can, &f);
}

// ---- 静态槽位缓冲区定义 ----
DjiC6xxMin::SlotBuf DjiC6xxMin::s_slot_bufs_[DjiC6xxMin::kMaxGroups] = {};

DjiC6xxMin::SlotBuf* DjiC6xxMin::FindOrAllocBuf(BspCanHandle can, uint16_t tx_std_id)
{
    // 先查已存在的
    for (int i = 0; i < kMaxGroups; ++i) {
        if (s_slot_bufs_[i].can == can && s_slot_bufs_[i].tx_std_id == tx_std_id) {
            return &s_slot_bufs_[i];
        }
    }
    // 分配新槽
    for (int i = 0; i < kMaxGroups; ++i) {
        if (s_slot_bufs_[i].can == nullptr) {
            s_slot_bufs_[i].can = can;
            s_slot_bufs_[i].tx_std_id = tx_std_id;
            return &s_slot_bufs_[i];
        }
    }
    return nullptr; // 表已满（不应发生）
}

void DjiC6xxMin::UpdateSlot()
{
    if (!can_) return;
    // slot 编号由 rx_std_id 相对 tx_std_id 的偏移决定：
    //   rx=0x201, tx=0x200 → slot 0
    //   rx=0x202, tx=0x200 → slot 1  ...以此类推
    if (cfg_.rx_std_id <= cfg_.tx_std_id) return;
    const int slot = static_cast<int>(cfg_.rx_std_id - cfg_.tx_std_id) - 1;
    if (slot < 0 || slot >= kSlotsPerGroup) return;

    SlotBuf* buf = FindOrAllocBuf(can_, cfg_.tx_std_id);
    if (!buf) return;
    buf->slots[slot] = target_current_raw();
}

void DjiC6xxMin::FlushGroup(BspCanHandle can, uint16_t tx_std_id)
{
    if (!can) return;
    for (int i = 0; i < kMaxGroups; ++i) {
        if (s_slot_bufs_[i].can == can && s_slot_bufs_[i].tx_std_id == tx_std_id) {
            const SlotBuf& b = s_slot_bufs_[i];
            BspCanFrame f = MakeGroupCurrentFrame(tx_std_id,
                                                  b.slots[0], b.slots[1],
                                                  b.slots[2], b.slots[3]);
            bsp_can_send(can, &f);
            return;
        }
    }
}

} // namespace actuator::drivers
