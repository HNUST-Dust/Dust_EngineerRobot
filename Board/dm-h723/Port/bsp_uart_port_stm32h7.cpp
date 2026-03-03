#include "bsp_uart_port.h"

// Board / HAL
#include "usart.h"

#include <cstring>

// ---- TX 环形缓冲区 ----
// 大小必须是 2 的幂，head/tail 用 uint16_t 自然溢出实现无锁环绕
static constexpr uint16_t kTxBufSize = 1024;

struct TxRingBuf {
    uint8_t  buf[kTxBufSize]{};
    uint16_t head     = 0;          // 写指针（只由 bsp_uart_send 写）
    uint16_t tail     = 0;          // 读指针（只由 TxCplt 中断写）
    bool     dma_busy = false;      // DMA 正在传输中

    uint16_t free() const {
        return static_cast<uint16_t>(kTxBufSize - 1u
               - static_cast<uint16_t>((head - tail) & (kTxBufSize - 1u)));
    }
    uint16_t used() const {
        return static_cast<uint16_t>((head - tail) & (kTxBufSize - 1u));
    }
};

struct BspUartOpaque {
    UART_HandleTypeDef* huart;
    BspUartRxCallback   cb;
    uint8_t*            rx_buf;
    uint16_t            rx_len;
    TxRingBuf           tx;
};

static uint8_t s_uart1_rx[512];
static uint8_t s_uart5_rx[512];
static uint8_t s_uart7_rx[512];
static uint8_t s_uart10_rx[512];

static BspUartOpaque s_uart1 {&huart1,  nullptr, s_uart1_rx,  0, {}};
static BspUartOpaque s_uart5 {&huart5,  nullptr, s_uart5_rx,  0, {}};
static BspUartOpaque s_uart7 {&huart7,  nullptr, s_uart7_rx,  0, {}};
static BspUartOpaque s_uart10{&huart10, nullptr, s_uart10_rx, 0, {}};

static inline BspUartOpaque* to_impl(BspUartHandle h) {
    return reinterpret_cast<BspUartOpaque*>(h);
}

static inline BspUartOpaque* from_hal(UART_HandleTypeDef* huart)
{
    if (huart == &huart1)  return &s_uart1;
    if (huart == &huart5)  return &s_uart5;
    if (huart == &huart7)  return &s_uart7;
    if (huart == &huart10) return &s_uart10;
    return nullptr;
}

BspUartHandle bsp_uart_get(BspUartId id)
{
    switch (id)
    {
        case BSP_UART1:   return reinterpret_cast<BspUartHandle>(&s_uart1);
        case BSP_UART5:   return reinterpret_cast<BspUartHandle>(&s_uart5);
        case BSP_UART7:   return reinterpret_cast<BspUartHandle>(&s_uart7);
        case BSP_USART10: return reinterpret_cast<BspUartHandle>(&s_uart10);
        default: return nullptr;
    }
}

static void start_rx(BspUartOpaque* impl)
{
    if (!impl || !impl->huart || !impl->rx_buf || impl->rx_len == 0) return;

    // Restart DMA RxToIdle
    (void)HAL_UARTEx_ReceiveToIdle_DMA(impl->huart, impl->rx_buf, impl->rx_len);

    // Avoid Half Transfer interrupts (optional but usually desired)
    if (impl->huart->hdmarx) {
        __HAL_DMA_DISABLE_IT(impl->huart->hdmarx, DMA_IT_HT);
    }
}

void bsp_uart_init(BspUartHandle h, BspUartRxCallback cb, uint16_t rx_buffer_length)
{
    auto* impl = to_impl(h);
    if (!impl) return;

    impl->cb = cb;

    // Clamp to internal buffer size
    constexpr uint16_t kBufSize = 512;
    impl->rx_len = (rx_buffer_length == 0) ? 0 : (rx_buffer_length > kBufSize ? kBufSize : rx_buffer_length);

    // 初始化 TX 环形缓冲区
    impl->tx.head     = 0;
    impl->tx.tail     = 0;
    impl->tx.dma_busy = false;

    start_rx(impl);
}

// 从环形缓冲区启动一次 DMA 发送（只在中断或关中断上下文调用）
static void start_tx_dma(BspUartOpaque* impl)
{
    TxRingBuf& tx = impl->tx;
    if (tx.dma_busy) return;

    const uint16_t avail = tx.used();
    if (avail == 0) return;

    // 计算本次线性可发送长度（不跨越缓冲区末尾）
    const uint16_t tail_idx = static_cast<uint16_t>(tx.tail & (kTxBufSize - 1u));
    const uint16_t to_end   = static_cast<uint16_t>(kTxBufSize - tail_idx);
    const uint16_t len      = (avail < to_end) ? avail : to_end;

    tx.dma_busy = true;
    (void)HAL_UART_Transmit_DMA(impl->huart, &tx.buf[tail_idx], len);
}

// bsp_uart_send：将数据写入 TX 环形缓冲区；若 DMA 空闲则立即启动
bool bsp_uart_send(BspUartHandle h, uint8_t* data, uint16_t length)
{
    auto* impl = to_impl(h);
    if (!impl || !impl->huart || !data || length == 0) return false;

    TxRingBuf& tx = impl->tx;

    // 缓冲区空间不足则丢弃（调用方负责分帧/限速）
    if (tx.free() < length) return false;

    // 写入环形缓冲区
    for (uint16_t i = 0; i < length; ++i) {
        tx.buf[tx.head & (kTxBufSize - 1u)] = data[i];
        ++tx.head;
    }

    // 若 DMA 空闲，立即启动（关中断保证原子）
    __disable_irq();
    start_tx_dma(impl);
    __enable_irq();

    return true;
}

// TX DMA 完成回调：推进 tail，继续发剩余数据
extern "C" void HAL_UART_TxCpltCallback(UART_HandleTypeDef* huart)
{
    auto* impl = from_hal(huart);
    if (!impl) return;

    TxRingBuf& tx = impl->tx;

    // 推进 tail：本次传输的字节数由 DMA 计数寄存器反推，
    // 但更简单的做法是记录本次发送长度。这里用 HAL 的 TxXferSize。
    const uint16_t sent = static_cast<uint16_t>(huart->TxXferSize);
    tx.tail = static_cast<uint16_t>(tx.tail + sent);
    tx.dma_busy = false;

    // 若缓冲区还有数据，继续发
    start_tx_dma(impl);
}

extern "C" void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef* huart, uint16_t Size)
{
    auto* impl = from_hal(huart);
    if (!impl) return;

    if (impl->cb) {
        // Size is number of received bytes at IDLE event
        impl->cb(impl->rx_buf, Size);
    }

    start_rx(impl);
}

extern "C" void HAL_UART_ErrorCallback(UART_HandleTypeDef* huart)
{
    auto* impl = from_hal(huart);
    if (!impl) return;

    start_rx(impl);
}
