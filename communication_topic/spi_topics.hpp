#pragma once

#include <cstdint>

#include "cmsis_os2.h"

#include "topic.hpp"

// 通用 SPI 事务 Topic：上层模块只描述“对哪个从机做什么事务”，
// 具体 SPI/HAL 调用由统一 SpiTxTask 串行执行（唯一出口）。
//
// 注意：该 Topic 允许携带指针/同步对象以实现 request/response（类似 RPC）。
// 指针生命周期必须覆盖事务完成（通常发布者会等待 done_evt）。

namespace orb {

enum class SpiDev : uint8_t {
    None = 0,

    // 与 Platform/bsp_spi_port.h 的 BspSpiDevId 数值保持一致，便于 Board/Drivers 映射。
    BMI088_ACCEL = 1,
    BMI088_GYRO  = 2,
};

enum class SpiOp : uint8_t {
    Select = 0,
    Deselect = 1,
    Transmit = 2,
    Receive = 3,
    Transceive = 4,
};

struct SpiTxFrame {
    SpiDev dev = SpiDev::None;
    SpiOp op = SpiOp::Transceive;

    const uint8_t* tx = nullptr;
    uint8_t* rx = nullptr;
    uint16_t len = 0;

    // 透传到底层 BSP 的超时（ms），仅对 BLOCK 模式有效。
    uint32_t timeout_ms = 1;

    // 可选同步：若提供，则 SpiTxTask 在完成后 set 事件位。
    osEventFlagsId_t done_evt = nullptr;
    uint32_t done_mask = 0;

    // 可选返回值：SpiTxTask 写入 true/false。
    bool* ok = nullptr;
};

inline RingTopic<SpiTxFrame, 64> spi_tx;

}  // namespace orb
