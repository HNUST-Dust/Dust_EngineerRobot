#include "dvc_pc_comm.h"

#include <cstdint>

#include "../communication_topic/pc_comm_topics.hpp"

void PcComm::Init(BspUsbHandle usb)
{
    usb_ = usb ? usb : bsp_usb_get();
    rx_buf_ = bsp_usb_get_rx_buffer();

    // 注册 BSP 回调：BSP 只负责透传 len，这里负责协议解析
    bsp_usb_init(usb_, &PcComm::UsbTxCallback, &PcComm::UsbRxCallback);

    // init 后再获取一次 buffer（底层可能在 init 中分配/返回）
    rx_buf_ = bsp_usb_get_rx_buffer();
}

void PcComm::UsbRxCallback(uint16_t len)
{
    PcComm::Instance().OnRx(len);
}

void PcComm::UsbTxCallback(uint16_t /*len*/)
{
    // 目前无需求；保留接口
}

void PcComm::OnRx(uint16_t len)
{
    (void)len;

    uint8_t* rx = rx_buf_ ? rx_buf_ : bsp_usb_get_rx_buffer();
    if (!rx) return;

    // 这里放协议解析（示例沿用你之前的 "SP" 头，只做最小发布）
    if (rx[0] == 'S' && rx[1] == 'P') {
        orb::PcRecvAutoAimData parsed{};
        orb::deserializePcRecv(rx, parsed);
        orb::pc_recv.publish(parsed);
    }
}
