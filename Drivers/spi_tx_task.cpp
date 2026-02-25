/**
 * @file spi_tx_task.cpp
 * @brief `Drivers/spi_tx_task.h` 的实现（drain `orb::spi_tx` 并调用 `bsp_spi_*()`）
 */

#include "spi_tx_task.h"

#include "bsp_spi_port.h"

#include "../communication_topic/spi_topics.hpp"

namespace {
BspSpiHandle to_bsp_handle(orb::SpiDev dev)
{
    switch (dev) {
        case orb::SpiDev::BMI088_ACCEL: return bsp_spi_get(BSP_SPI_DEV_BMI088_ACCEL);
        case orb::SpiDev::BMI088_GYRO:  return bsp_spi_get(BSP_SPI_DEV_BMI088_GYRO);
        default: return nullptr;
    }
}
}  // namespace

bool SpiTxTask::Start()
{
    if (started_) {
        configASSERT(false);
        return false;
    }

    started_ = true;

    evt_attr_ = osEventFlagsAttr_t{
        .name = "spi_tx_evt",
        .cb_mem = &evt_cb_,
        .cb_size = sizeof(evt_cb_),
    };
    evt_ = osEventFlagsNew(&evt_attr_);
    if (!evt_) {
        configASSERT(false);
        return false;
    }

    notifier_ = Notifier(evt_, kEvtBit);
    orb::spi_tx.register_notifier(&notifier_);

    const osThreadAttr_t attr{
        .name = "spi_tx",
        .cb_mem = &tcb_,
        .cb_size = sizeof(tcb_),
        .stack_mem = stack_,
        .stack_size = sizeof(stack_),
        .priority = (osPriority_t)osPriorityLow,
    };

    thread_ = osThreadNew(&SpiTxTask::TaskEntry, this, &attr);
    if (!thread_) {
        configASSERT(false);
        return false;
    }

    return true;
}

void SpiTxTask::TaskEntry(void* arg)
{
    static_cast<SpiTxTask*>(arg)->Task();
}

void SpiTxTask::Task()
{
    RingSub<orb::SpiTxFrame, 64> sub(orb::spi_tx);

    // 任务内缓存句柄，避免每次事务都查表。
    BspSpiHandle bmi088_accel = to_bsp_handle(orb::SpiDev::BMI088_ACCEL);
    BspSpiHandle bmi088_gyro = to_bsp_handle(orb::SpiDev::BMI088_GYRO);
    if (bmi088_accel) bsp_spi_set_mode(bmi088_accel, BSP_SPI_BLOCK_MODE);
    if (bmi088_gyro) bsp_spi_set_mode(bmi088_gyro, BSP_SPI_BLOCK_MODE);

    auto get_cached = [&](orb::SpiDev dev) -> BspSpiHandle {
        switch (dev) {
            case orb::SpiDev::BMI088_ACCEL: return bmi088_accel;
            case orb::SpiDev::BMI088_GYRO:  return bmi088_gyro;
            default: return nullptr;
        }
    };

    for (;;) {
        (void)osEventFlagsWait(evt_, kEvtBit, osFlagsWaitAny, 1);

        orb::SpiTxFrame pkt{};
        while (sub.copy(pkt)) {
            bool ok = false;

            BspSpiHandle h = get_cached(pkt.dev);
            if (h) {
                switch (pkt.op) {
                    case orb::SpiOp::Select:
                        bsp_spi_select(h);
                        ok = true;
                        break;
                    case orb::SpiOp::Deselect:
                        bsp_spi_deselect(h);
                        ok = true;
                        break;
                    case orb::SpiOp::Transmit:
                        ok = bsp_spi_transmit(h, pkt.tx, pkt.len, pkt.timeout_ms);
                        break;
                    case orb::SpiOp::Receive:
                        ok = bsp_spi_receive(h, pkt.rx, pkt.len, pkt.timeout_ms);
                        break;
                    case orb::SpiOp::Transceive:
                    default:
                        ok = bsp_spi_transceive(h, pkt.tx, pkt.rx, pkt.len, pkt.timeout_ms);
                        break;
                }
            }

            if (pkt.ok) {
                *pkt.ok = ok;
            }
            if (pkt.done_evt && pkt.done_mask != 0u) {
                (void)osEventFlagsSet(pkt.done_evt, pkt.done_mask);
            }
        }

        osDelay(1);
    }
}
