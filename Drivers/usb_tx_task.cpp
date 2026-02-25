#include "usb_tx_task.h"

#include "../daemon_supervisor/supervisor.hpp"
#include "../Platform/bsp_usb_port.h"

namespace {
inline uint32_t now_ms()
{
    return static_cast<uint32_t>(dwt_get_timeline_ms());
}

DaemonClient* s_usb_tx_daemon = nullptr;
} // namespace

bool UsbTxTask::Start()
{
    if (started_) return false;

    {
        static DaemonClient s_daemon(
            3000,
            nullptr,
            nullptr,
            DaemonClient::Domain::COMM,
            DaemonClient::FaultLevel::WARN,
            DaemonClient::Priority::HIGH);
        s_usb_tx_daemon = &s_daemon;
        (void)DaemonSupervisor::register_client(s_usb_tx_daemon);
        s_usb_tx_daemon->feed(now_ms());
    }

    started_ = true;

    evt_attr_ = osEventFlagsAttr_t{
        .name = "usb_tx_evt",
        .cb_mem = &evt_cb_,
        .cb_size = sizeof(evt_cb_),
    };
    evt_ = osEventFlagsNew(&evt_attr_);
    if (!evt_) {
        configASSERT(false);
        return false;
    }

    notifier_ = Notifier(evt_, kEvtBit);
    orb::pc_send.register_notifier(&notifier_);

    const osThreadAttr_t attr{
        .name = "usb_tx",
        .cb_mem = &tcb_,
        .cb_size = sizeof(tcb_),
        .stack_mem = stack_,
        .stack_size = sizeof(stack_),
        .priority = (osPriority_t)osPriorityLow,
    };

    thread_ = osThreadNew(&UsbTxTask::TaskEntry, this, &attr);
    if (!thread_) {
        configASSERT(false);
        return false;
    }
    return true;
}

void UsbTxTask::TaskEntry(void* arg)
{
    static_cast<UsbTxTask*>(arg)->Task();
}

void UsbTxTask::Task()
{
    Subscription<orb::PcSendAutoAimData> sub(orb::pc_send);

    for (;;) {
        (void)osEventFlagsWait(evt_, kEvtBit, osFlagsWaitAny, 1);

        orb::PcSendAutoAimData pending{};
        while (sub.copy(pending)) {
            uint8_t buffer[43];
            std::memset(buffer, 0, sizeof(buffer));
            orb::serializePcSend(pending, buffer);

            BspUsbHandle h = bsp_usb_get();
            if (!h) continue;

            if (s_usb_tx_daemon) s_usb_tx_daemon->feed(now_ms());

            (void)bsp_usb_transmit(h, buffer, static_cast<uint16_t>(43));
        }

        osDelay(1);
    }
}
