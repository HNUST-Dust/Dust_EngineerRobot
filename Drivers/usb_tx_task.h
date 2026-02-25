#pragma once

#include "cmsis_os2.h"

#include "FreeRTOS.h"
#include "task.h"

#include "bsp_dwt.h"
#include "../communication_topic/topic_notify.hpp"
#include "../communication_topic/pc_comm_topics.hpp"

class UsbTxTask {
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
