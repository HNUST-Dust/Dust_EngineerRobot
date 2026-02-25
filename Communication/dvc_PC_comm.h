#pragma once
#include "debug_tools.h"
#include "cmsis_os2.h"
#include <cstdint>
#include "../communication_topic/pc_comm_topics.hpp"

class PcComm {
public:
    static PcComm& Instance() {
        static PcComm instance;
        return instance;
    }

    struct Config {
    };

    orb::PcSendAutoAimData pc_send_data_;
    orb::PcRecvAutoAimData pc_recv_data_;

    void Init(const Config& cfg = {});
    void Task();
    void RxCpltCallback(uint16_t len);
    
    private:
    DebugTools debug_tools_;
    bool started_ = false;
    osThreadId_t thread_ = nullptr;
    
    Config cfg_{};
    
    static void TaskEntry(void *param);
};