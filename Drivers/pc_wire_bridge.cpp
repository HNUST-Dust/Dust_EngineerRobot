#include "pc_wire_bridge.h"

#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "task.h"

#include "../communication_topic/pc_comm_topics.hpp"
#include "bsp_dwt.h"
#include "../daemon_supervisor/supervisor.hpp"

namespace {
    DaemonClient* s_bridge_daemon = nullptr;
    inline uint32_t now_ms() { return static_cast<uint32_t>(dwt_get_timeline_ms()); }
}

bool PcWireBridge::Start()
{
    // No-op start: bridge no longer runs a dedicated task. Keep started_ flag.
    if (started_) return false;
    started_ = true;
    return true;
}
void PcWireBridge::DeserializeAndPublish(const uint8_t* buf, size_t len)
{
    if (!buf || len < 29) return;
    orb::PcRecvAutoAimData parsed{};
    orb::deserializePcRecv(buf, parsed);
    orb::pc_recv.publish(parsed);
}
