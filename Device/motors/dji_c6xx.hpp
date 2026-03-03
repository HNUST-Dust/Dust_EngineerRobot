#pragma once

#include <cstdint>

#include "bsp_can_port.h"

namespace actuator::drivers {

class DjiC6xxMin final {
public:
    struct Config {
        // 纯协议层：保留 bus 字段供上层索引（不依赖 Topic 系统）
        uint8_t bus = 1; // 1->CAN1, 2->CAN2 (由上层自行约定)

        // 接收反馈帧 ID（0x201..0x204）
        uint16_t rx_std_id = 0x201;

        // 发送组电流帧 ID（通常为 0x200，另一组可为 0x1FF）
        uint16_t tx_std_id = 0x200;

        float gearbox_ratio = 1.0f;

        float current_limit = 20.0f;
        uint16_t enc_per_round = 8192;
    };

    // Init 时自动将自身注册到 (can, tx_std_id) 对应的发送组，slot 由 rx_std_id - tx_std_id 确定
    void Init(BspCanHandle can, const Config& cfg);

    // CAN RX complete callback entry (called from bus-level ID dispatch)
    void CanRxCpltCallback(const BspCanFrame* frame);

    // 输入：外部控制器计算后的电流(A)
    void SetTargetCurrent(float current);

    // 输出：DJI 组帧使用的 raw 电流（16-bit signed, big-endian per slot）
    int16_t target_current_raw() const;

    // 反馈状态（解析自反馈帧，单位：rad / rad/s / A / ℃）
    float now_angle_rad() const { return now_angle_; }
    float now_omega_rad_s() const { return now_omega_out_; }
    float now_current_a() const { return now_current_; }
    float temperature_c() const { return temperature_; }

    // 目标状态（单位：A）
    float target_current_a() const { return target_current_; }

    uint8_t bus() const { return cfg_.bus; }
    uint16_t rx_id() const { return cfg_.rx_std_id; }
    uint16_t tx_id() const { return cfg_.tx_std_id; }

    // 聚合最多4个同组电机的电流，使用第一个非空电机持有的 can 发送一帧
    // 由 App 层显式传入同组电机，避免自动分组时的跨模块干扰
    // 用法：DjiC6xxMin::SendGroup(&m1, &m2, &m3, &m4);
    static void SendGroup(DjiC6xxMin* m0,
                          DjiC6xxMin* m1 = nullptr,
                          DjiC6xxMin* m2 = nullptr,
                          DjiC6xxMin* m3 = nullptr);

    // ---- 分步发送接口（用于跨任务共享同一 CAN 帧的场景）----
    //
    // UpdateSlot()：将本电机的 target_current_raw() 写入静态槽位缓冲区。
    //   slot 由 (rx_std_id - tx_std_id - 1) 推算，值域 [0, 3]。
    //   可在任意任务中调用，仅写内存，不触发 CAN 发送。
    //
    // FlushGroup()：读取指定 (can, tx_std_id) 对应缓冲区，打包为一帧后发送。
    //   应在唯一的"发送方"任务中调用（每组每周期调用一次）。
    //
    // 典型用法（CAN3/0x200 组，y_axis 在 Gantry 任务，wrist 在 Arm 任务）：
    //   Arm 任务：  g_wrist_joint_left.UpdateSlot();
    //               g_wrist_joint_right.UpdateSlot();
    //   Gantry任务：g_motor_y_axis.UpdateSlot();
    //               DjiC6xxMin::FlushGroup(can3, 0x200);  // 唯一发送方
    void UpdateSlot();
    // 以 (can, tx_std_id) 为 key 发送对应缓冲区中的完整帧
    static void FlushGroup(BspCanHandle can, uint16_t tx_std_id);
    // 便捷重载：以某个电机实例持有的 (can_, tx_std_id) 为 key
    static void FlushGroup(const DjiC6xxMin& ref) { FlushGroup(ref.can_, ref.cfg_.tx_std_id); }

    // ---- 纯协议层工具函数：DJI C6xx 组电流帧（0x200 / 0x1FF 等）----
    // 打包 4 路 raw current 到 8 bytes（每路 int16, big-endian）。
    static void PackGroupCurrent(int16_t i0, int16_t i1, int16_t i2, int16_t i3, uint8_t out[8]);

    // 直接构造一个 CAN 标准帧（len=8）。
    static BspCanFrame MakeGroupCurrentFrame(uint16_t tx_std_id,
                                             int16_t i0, int16_t i1, int16_t i2, int16_t i3);

    // 兼容旧接口：直接发送组电流帧（若 can 为空则不发送）。
    static void SendGroupCurrent(BspCanHandle can, uint16_t tx_std_id,
                                 int16_t i0, int16_t i1, int16_t i2, int16_t i3);

private:
    static constexpr float k2pi = 6.283185307179586f;

    // ---- 静态槽位缓冲区（用于 UpdateSlot / FlushGroup）----
    // 最多支持 4 条总线 × 2 个 tx_std_id（0x200 / 0x1FF）× 4 个 slot
    // key = (bus-1)*2 + (tx_std_id==0x1FF ? 1 : 0)，共 8 个"组"
    static constexpr int kMaxGroups = 8;
    static constexpr int kSlotsPerGroup = 4;
    struct SlotBuf {
        BspCanHandle can   = nullptr;
        uint16_t tx_std_id = 0;
        int16_t  slots[kSlotsPerGroup] = {};
    };
    static SlotBuf s_slot_bufs_[kMaxGroups];

    // 根据 (bus, tx_std_id) 查找或分配一个 SlotBuf，返回 nullptr 表示已满
    static SlotBuf* FindOrAllocBuf(BspCanHandle can, uint16_t tx_std_id);

    Config cfg_{};
    BspCanHandle can_ = nullptr;

    uint16_t last_enc_ = 0;
    int32_t total_round_ = 0;
    float now_angle_ = 0.0f;
    float now_omega_out_ = 0.0f;
    float now_current_ = 0.0f;
    float temperature_ = 0.0f;

    float target_current_ = 0.0f;
};

} // namespace actuator::drivers
