#pragma once
#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

typedef struct BspUsbOpaque* BspUsbHandle;

typedef void (*BspUsbCallback)(uint16_t len);

// 获取全局 USB 设备句柄（实现可返回单例）
BspUsbHandle bsp_usb_get();

// 初始化 USB BSP：注册回调并分配 RX buffer
void bsp_usb_init(BspUsbHandle h, BspUsbCallback tx_cb, BspUsbCallback rx_cb);

// 非阻塞发送（与底层 HAL 的语义保持一致）
bool bsp_usb_transmit(BspUsbHandle h, const uint8_t* data, uint16_t length);

// 获取底层接收缓冲区指针
uint8_t* bsp_usb_get_rx_buffer();

#ifdef __cplusplus
}
#endif
