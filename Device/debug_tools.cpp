#include "debug_tools.h"

#include <cstdint>
#include <cstring>

#include "cmsis_os2.h"

DebugTools& DebugTools::Instance()
{
    static DebugTools inst;
    return inst;
}

void DebugTools::Init(BspUartHandle uart)
{
    if (started_) {
        return;
    }
    if (uart == nullptr) {
        return;
    }

    uart_ = uart;
    started_ = true;
}

void DebugTools::StartThread()
{
    if (!started_ || uart_ == nullptr) {
        return;
    }
    if (thread_ != nullptr) {
        return;
    }

    static const osThreadAttr_t kDebugToolsTaskAttr = {
        .name = "debug_tools",
        .stack_size = 512,
        .priority = (osPriority_t)osPriorityNormal,
    };
    thread_ = osThreadNew(DebugTools::TaskEntry, this, &kDebugToolsTaskAttr);
}

void DebugTools::FeedFloat(float v)
{
    // lock-free: 只允许单调增加计数；超过容量直接丢弃
    uint32_t idx = float_count_.load(std::memory_order_relaxed);
    if (idx >= kMaxFloats) {
        return;
    }
    floats_[idx] = v;
    float_count_.store(idx + 1, std::memory_order_release);
}

void DebugTools::TaskEntry(void* argument)
{
    auto* self = static_cast<DebugTools*>(argument);
    self->Task();
}

void DebugTools::Task()
{
    for (;;) {
        // // 1) 订阅底盘四轮转速 topic
        // ChassisWheelOmega omega{};
        // if (chassis_omega_sub_.copy(omega)) {
        //     VofaSendFloat(omega.omega_1_rad_s);
        //     VofaSendFloat(omega.omega_2_rad_s);
        //     VofaSendFloat(omega.omega_3_rad_s);
        //     VofaSendFloat(omega.omega_4_rad_s);
        //     VofaSendTail();
        // }

        bool any_state_updated = false;

        // 2) 订阅龙门架/机械臂状态（原 Robot::Task 调试输出）
        orb::GantryState gantry_st{};
        if (gantry_state_sub_.copy(gantry_st)) {
            any_state_updated = true;
            // X/Y 目标速度 vs 当前速度
            VofaSendFloat(gantry_st.x_target_omega);
            VofaSendFloat(gantry_st.x_left_now_omega);
            VofaSendFloat(gantry_st.y_target_omega);
            VofaSendFloat(gantry_st.y_now_omega);
            VofaSendFloat(gantry_st.z_target_angle);
            VofaSendFloat(gantry_st.z_left_now_angle);
            VofaSendFloat(gantry_st.z_left_omega);
            VofaSendFloat(gantry_st.z_left_torque);
            VofaSendTail();
        }

        // orb::ArmState arm_st{};
        // if (arm_state_sub_.copy(arm_st)) {
        //     any_state_updated = true;
        //     VofaSendFloat(arm_st.claws_virtual_angle);
        //     VofaSendFloat(arm_st.elbow_pitch_virtual_angle);
        //     VofaSendFloat(arm_st.elbow_yaw_virtual_angle);
        //     VofaSendFloat(arm_st.wrist_left_virtual_angle);
        //     VofaSendFloat(arm_st.wrist_right_virtual_angle);
        // }

        // if (any_state_updated) {
        //     VofaSendTail();
        // }

        // // 3) 兼容外部 FeedFloat 缓冲（可选）
        // const uint32_t n = float_count_.exchange(0, std::memory_order_acq_rel);
        // const uint32_t send_n = (n > kMaxFloats) ? kMaxFloats : n;

        // for (uint32_t i = 0; i < send_n; ++i) {
        //     VofaSendFloat(floats_[i]);
        // }
        // if (send_n > 0) {
        //     VofaSendTail();
        // }

        osDelay(10); // 100Hz → 每帧 68B，115200bps 约 5.9ms 发完，不积压
    }
}

void DebugTools::VofaSendFloat(float data)
{
    if (!started_ || uart_ == nullptr) {
        return;
    }

    static_assert(sizeof(float) == 4, "VOFA float must be 4 bytes");
    uint8_t buf[4]{};
    std::memcpy(buf, &data, 4);
    (void)bsp_uart_send(uart_, buf, 4);
}

void DebugTools::VofaSendTail()
{
    if (!started_ || uart_ == nullptr) {
        return;
    }

    uint8_t tail[4] = {0x00, 0x00, 0x80, 0x7f};
    (void)bsp_uart_send(uart_, tail, 4);
}

void DebugTools::VofaReceiveCallback(uint8_t *buffer, uint16_t length)
{
    (void)buffer;
    (void)length;
}