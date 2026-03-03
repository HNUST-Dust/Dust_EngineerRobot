#pragma once

#include <cstdint>

#include "bsp_usb_port.h"

// PC 通信（上层驱动/协议层）：
// - 依赖 BSP USB 端口获取 RX buffer，并在 RX 回调中解析协议。
// - 解析后的数据通过 communication_topic 发布（话题/协议在上层，BSP 不应包含）。

class PcComm {
public:
    static PcComm& Instance() {
        static PcComm inst;
        return inst;
    }

    void Init(BspUsbHandle usb = nullptr);

private:
    PcComm() = default;
    PcComm(const PcComm&) = delete;
    PcComm& operator=(const PcComm&) = delete;

    static void UsbRxCallback(uint16_t len);
    static void UsbTxCallback(uint16_t len);

    void OnRx(uint16_t len);

    BspUsbHandle usb_ = nullptr;
    uint8_t* rx_buf_ = nullptr;
};
