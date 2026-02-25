#pragma once

#include "../communication_topic/pc_comm_topics.hpp"

// PcWireBridge retained as a helper class but WITHOUT an internal task.
// It no longer creates threads or registers notifiers; driver-side code
// should call its helpers if needed. Kept minimal for future reuse.
class PcWireBridge {
public:
    // Lightweight init which does not start any thread.
    bool Start();
    void Init() { (void)Start(); }

    // Helper: deserialize a packed wire buffer (29 bytes) and publish
    // the high-level topic. Useful if some code wants to reuse bridge logic.
    static void DeserializeAndPublish(const uint8_t* buf, size_t len);

private:
    bool started_ = false;
};
