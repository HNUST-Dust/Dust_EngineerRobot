#pragma once

#include <cstdint>

namespace alg {
class Pid;
}

// UART PID tuner (ASCII line protocol).
//
// Each command is a short text line ending with '\n' (CRLF is also accepted).
// Example bytes: "kp:1.23456\n"  ->  6B 70 3A 31 2E 32 33 34 35 36 0A
// Value precision: parsed & quantized to 5 decimal places (1e-5).
//
// Supported keys:
//   - pid:<name|slot>   Select target PID for subsequent set/reset.
//                       Targets are registered by firmware via RegisterTarget().
//                       <slot> can be decimal ("1") or hex ("0x01") (1-based).
//   - kp:<float>
//   - ki:<float>
//   - kd:<float>
//   - kf:<float>
//   - i_out_max:<float>
//   - out_max:<float>
//   - reset:<0/1>       When value != 0, reset currently selected PID.
//
// Internally this module converts lines into `PidTunerMsg` and pushes them into a FreeRTOS queue.
// Parsing happens in UART Rx callback context (ISR); application happens in task context.

struct PidTunerMsg {
    uint8_t cmd = 0;
    uint8_t flags = 0;
    uint8_t arg = 0;      // kCmdSet: field_id; kCmdSelect: target index
    uint8_t reserved = 0;
    float value = 0.0f;
    uint16_t seq = 0;
};

class PidTuner {
public:
    void Init();

    // Register a tunable PID target. Must be called from task context.
    // `name` is used by host command: "pid:<name>\n".
    bool RegisterTarget(const char* name, alg::Pid* pid);

    // Called from UART Rx callback context (ISR).
    void OnRx(const uint8_t* data, uint16_t len);

    // Called from task context.
    // Returns true if a message is popped.
    bool Pop(PidTunerMsg& out);

    // Convenience: process all pending messages and apply to the currently selected PID.
    // Must be called from the same task that owns/uses the registered PID instances.
    void ProcessInTask();

    static constexpr uint8_t kCmdSet = 0x01;
    static constexpr uint8_t kCmdReset = 0x02;
    static constexpr uint8_t kCmdSelect = 0x03;

    static constexpr uint8_t kFlagResetAfterSet = 1u << 0;

private:
    static constexpr uint16_t kAccumCap = 128;
    static constexpr uint8_t kMaxTargets = 16;

    void TryParseLinesFromAccum();

    struct Target {
        const char* name = nullptr;
        alg::Pid* pid = nullptr;
    };

    void* queue_ = nullptr; // QueueHandle_t (opaque here to avoid FreeRTOS headers in consumers)

    uint8_t accum_[kAccumCap]{};
    uint16_t accum_len_ = 0;

    Target targets_[kMaxTargets]{};
    uint8_t target_count_ = 0;
    uint8_t selected_target_ = 0; // index in targets_

    uint16_t seq_ = 0;
};

extern PidTuner g_pid_tuner;
