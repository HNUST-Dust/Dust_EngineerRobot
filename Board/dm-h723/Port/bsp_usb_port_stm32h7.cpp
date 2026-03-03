// Board-specific USB port implementation for STM32H7 (combines Drivers-level USB
// functions and port-level API to match UART/CAN design).

#include "bsp_usb_port.h"
#include "../Inc/usbd_cdc_if.h"

struct BspUsbOpaque {
    BspUsbCallback tx_cb;
    BspUsbCallback rx_cb;
};

static BspUsbOpaque g_usb_impl;
static uint8_t* s_rx_buffer = nullptr;

BspUsbHandle bsp_usb_get() {
    return &g_usb_impl;
}

static void c_tx_trampoline(uint16_t len) {
    if (g_usb_impl.tx_cb) g_usb_impl.tx_cb(len);
}

static void c_rx_trampoline(uint16_t len) {
    // BSP 层只做回调透传，不做任何协议解析/话题发布
    if (g_usb_impl.rx_cb) g_usb_impl.rx_cb(len);
}

// Port-level API: 直接使用底层 HAL 接口，不再导出旧的 USB_* 全局/函数
void bsp_usb_init(BspUsbHandle h, BspUsbCallback tx_cb, BspUsbCallback rx_cb) {
    if (!h) h = &g_usb_impl;
    h->tx_cb = tx_cb;
    h->rx_cb = rx_cb;
    // Register callbacks with HAL and obtain RX buffer
    s_rx_buffer = CDCInitRxbufferNcallback((UsbCallback)c_tx_trampoline, (UsbCallback)c_rx_trampoline);
}

bool bsp_usb_transmit(BspUsbHandle h, const uint8_t* data, uint16_t length) {
    if (!data || length == 0) return false;
    CDC_Transmit_HS((uint8_t*)data, length);
    return true;
}

uint8_t* bsp_usb_get_rx_buffer() {
    return s_rx_buffer;
}
