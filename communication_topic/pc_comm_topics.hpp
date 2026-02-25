#pragma once
#include "topic.hpp"
#include <cstdint>
#include <cstring>

namespace orb {
// Use natural alignment for in-memory structures to avoid unaligned access
// during processing. We will explicitly serialize to wire format before send.
struct PcSendAutoAimData {
    uint8_t head[2] = {'S', 'P'};
    uint8_t mode = 0;
    float q[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    struct {
        float yaw_ang;
        float yaw_vel;
    } yaw{};
    struct {
        float pitch_ang;
        float pitch_vel;
    } pitch{};
    struct {
        float bullet_speed;
        uint16_t bullet_count;
    } bullet{};
    uint16_t crc16 = 0;
};

struct PcRecvAutoAimData {
    uint8_t head[2] = {'S', 'P'};
    uint8_t mode = 0;
    struct {
        float yaw_ang;
        float yaw_vel;
        float yaw_acc;
    } yaw{};
    struct {
        float pitch_ang;
        float pitch_vel;
        float pitch_acc;
    } pitch{};
    uint16_t crc16 = 0;
};

// Serialize PcSendAutoAimData into a packed wire buffer (43 bytes).
// Caller must provide a buffer of at least 43 bytes.
inline void serializePcSend(const PcSendAutoAimData& src, uint8_t* dst)
{
    // Layout (bytes): head[2], mode(1), q[4]*4, yaw{2*4}, pitch{2*4}, bullet_speed(4), bullet_count(2), crc16(2)
    size_t pos = 0;
    std::memcpy(dst + pos, src.head, 2); pos += 2;
    std::memcpy(dst + pos, &src.mode, 1); pos += 1;
    std::memcpy(dst + pos, src.q, sizeof(src.q)); pos += sizeof(src.q);
    std::memcpy(dst + pos, &src.yaw.yaw_ang, sizeof(src.yaw.yaw_ang)); pos += sizeof(src.yaw.yaw_ang);
    std::memcpy(dst + pos, &src.yaw.yaw_vel, sizeof(src.yaw.yaw_vel)); pos += sizeof(src.yaw.yaw_vel);
    std::memcpy(dst + pos, &src.pitch.pitch_ang, sizeof(src.pitch.pitch_ang)); pos += sizeof(src.pitch.pitch_ang);
    std::memcpy(dst + pos, &src.pitch.pitch_vel, sizeof(src.pitch.pitch_vel)); pos += sizeof(src.pitch.pitch_vel);
    std::memcpy(dst + pos, &src.bullet.bullet_speed, sizeof(src.bullet.bullet_speed)); pos += sizeof(src.bullet.bullet_speed);
    std::memcpy(dst + pos, &src.bullet.bullet_count, sizeof(src.bullet.bullet_count)); pos += sizeof(src.bullet.bullet_count);
    std::memcpy(dst + pos, &src.crc16, sizeof(src.crc16)); pos += sizeof(src.crc16);
}

// Deserialize a packed 29-byte wire buffer into PcRecvAutoAimData (host in-memory layout)
inline void deserializePcRecv(const uint8_t* src, PcRecvAutoAimData& dst)
{
    size_t pos = 0;
    std::memcpy(dst.head, src + pos, 2); pos += 2;
    std::memcpy(&dst.mode, src + pos, 1); pos += 1;
    std::memcpy(&dst.yaw.yaw_ang, src + pos, sizeof(dst.yaw.yaw_ang)); pos += sizeof(dst.yaw.yaw_ang);
    std::memcpy(&dst.yaw.yaw_vel, src + pos, sizeof(dst.yaw.yaw_vel)); pos += sizeof(dst.yaw.yaw_vel);
    std::memcpy(&dst.yaw.yaw_acc, src + pos, sizeof(dst.yaw.yaw_acc)); pos += sizeof(dst.yaw.yaw_acc);
    std::memcpy(&dst.pitch.pitch_ang, src + pos, sizeof(dst.pitch.pitch_ang)); pos += sizeof(dst.pitch.pitch_ang);
    std::memcpy(&dst.pitch.pitch_vel, src + pos, sizeof(dst.pitch.pitch_vel)); pos += sizeof(dst.pitch.pitch_vel);
    std::memcpy(&dst.pitch.pitch_acc, src + pos, sizeof(dst.pitch.pitch_acc)); pos += sizeof(dst.pitch.pitch_acc);
    std::memcpy(&dst.crc16, src + pos, sizeof(dst.crc16)); pos += sizeof(dst.crc16);
}

inline Topic<PcSendAutoAimData> pc_send;
inline Topic<PcRecvAutoAimData> pc_recv;

// Wire-level frames used to decouple USB transport from higher-level topics.
struct PcWireTx {
    uint16_t len = 0;
    uint8_t bytes[43] = {0};
};

struct PcWireRx {
    uint16_t len = 0;
    uint8_t bytes[29] = {0};
};

// Use RingTopic for wire frames to preserve small history and to wake TxTask.
inline RingTopic<PcWireTx, 8> pc_wire_tx;
inline RingTopic<PcWireRx, 8> pc_wire_rx;

} // namespace orb