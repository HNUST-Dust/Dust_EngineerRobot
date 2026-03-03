#ifndef DEBUG_TOOLS_H_
#define DEBUG_TOOLS_H_

#include <cstdint>
#include "bsp_uart_port.h"

#include <atomic>

#include "../communication_topic/chassis_topics.hpp"
#include "../communication_topic/gantry_state_topics.hpp"
#include "../communication_topic/arm_state_topics.hpp"

class DebugTools
{
private:
    BspUartHandle uart_ = nullptr;
    bool started_ = false;

    // 内部发送线程
    void* thread_ = nullptr;
    static void TaskEntry(void* argument);
    void Task();

    // 这里用简单的共享缓冲：由其它模块/回调写入，线程周期性发送。
    // 约束：每次最多发送 32 个 float。
    static constexpr uint32_t kMaxFloats = 32;
    std::atomic<uint32_t> float_count_{0};
    float floats_[kMaxFloats] = {0};

    // Topic 订阅（只关心最新值）
    Subscription<ChassisWheelOmega> chassis_omega_sub_{chassis_wheel_omega};
    Subscription<orb::GantryState> gantry_state_sub_{orb::gantry_state};
    Subscription<orb::ArmState> arm_state_sub_{orb::arm_state};

public:
    // Singleton accessor (explicit call-site; no global free-function)
    static DebugTools& Instance();

    // 绑定 UART 句柄（通过 bsp_uart_get 获取）
    void Init(BspUartHandle uart);

    // 创建线程并开始发送
    void StartThread();

    // 供其它模块调用：推送一个要发送的 float（满了会丢弃）
    void FeedFloat(float v);

    // 兼容旧调用：立即发送一次（直接走 bsp_uart_send）
    void VofaSendFloat(float data);
    void VofaSendTail();

    // RX 回调（如果你用它做“在线判据/喂狗”，就在这里处理）
    void VofaReceiveCallback(uint8_t *buffer, uint16_t length);
};

#endif // DEBUG_TOOLS_H_
