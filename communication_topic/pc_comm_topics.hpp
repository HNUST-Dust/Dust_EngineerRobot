#pragma once

#include <cstdint>
#include "topic.hpp"

namespace orb {

// Minimal PC communication topics used by USB port.
// Note: Only fields required by current code are defined.

struct PcRecvAutoAimData {
    // Add fields as needed by your desktop/vision protocol.
    // Keep it minimal to avoid unnecessary coupling.
    uint8_t dummy = 0;
};

inline Topic<PcRecvAutoAimData> pc_recv;

// Deserialize raw packet into PcRecvAutoAimData.
// Current USB port only needs this symbol to link; parsing can be extended later.
inline void deserializePcRecv(const uint8_t* /*buf*/, PcRecvAutoAimData& out)
{
    out = PcRecvAutoAimData{};
}

} // namespace orb
