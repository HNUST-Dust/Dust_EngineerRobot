/**
 * @file spi_tx_task.h
 * @brief 统一 SPI 事务任务（Topic -> BSP SPI）
 *
 * 设计思路：
 * =========
 * - 订阅 `orb::spi_tx`（通用 SPI 事务 Topic），串行执行所有 SPI 操作。
 * - 通过 Notifier/EventFlags 在有新事务时唤醒，避免空转。
 * - 该任务是全仓 SPI 访问的唯一出口：业务层/Device 层禁止直接调用 `bsp_spi_*`。
 */

#pragma once

#include "cmsis_os2.h"

#include "FreeRTOS.h"
#include "task.h"

#include "../communication_topic/topic_notify.hpp"

class SpiTxTask {
public:
    bool Start();

    void Init() { (void)Start(); }

private:
    static void TaskEntry(void* arg);
    void Task();

    bool started_ = false;

    osThreadId_t thread_ = nullptr;

    osEventFlagsId_t evt_ = nullptr;
    StaticEventGroup_t evt_cb_{};
    osEventFlagsAttr_t evt_attr_{};

    static constexpr uint32_t kEvtBit = 1u << 0;

    Notifier notifier_{nullptr, 0};

    StaticTask_t tcb_{};
    StackType_t stack_[512]{};
};
