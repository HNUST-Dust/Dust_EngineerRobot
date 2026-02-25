#pragma once
#include <cstdint>
#include "bsp_uart_port.h"
#include "../communication_topic/uart_topics.hpp"

struct RecviedRawData {
    uint8_t start_of_frame_1;
    uint8_t start_of_frame_2;
    uint64_t channel_0 : 11;
    uint64_t channel_1 : 11;
    uint64_t channel_2 : 11;
    uint64_t channel_3 : 11;
    uint64_t mode_switch : 2;
    uint64_t pause : 1;
    uint64_t fn_1 : 1;
    uint64_t fn_2 : 1;
    uint64_t wheel : 11;
    uint64_t trigger : 1;
    uint64_t reserved_1 : 3;
    int16_t mouse_x;
    int16_t mouse_y;
    int16_t mouse_z;
    uint8_t mouse_left_key : 2;
    uint8_t mouse_right_key : 2;
    uint8_t mouse_middle_key : 2;
    uint8_t reserved_2 : 2;
    union
    {
        uint16_t key_code;
        struct
        {
            uint16_t W : 1;
            uint16_t S : 1;
            uint16_t A : 1;
            uint16_t D : 1;
            uint16_t SHIFT : 1;
            uint16_t CTRL : 1;
            uint16_t Q : 1;
            uint16_t E : 1;
            uint16_t R : 1;
            uint16_t F : 1;
            uint16_t G : 1;
            uint16_t Z : 1;
            uint16_t X : 1;
            uint16_t C : 1;
            uint16_t V : 1;
            uint16_t B : 1;
        } bit;
    } keyboard;
    uint16_t crc16;
} __attribute__((packed));


class VT03{
public:
    static VT03& Instance() {
        static VT03 instance;
        return instance;
    }

    void Init(BspUartHandle uart, orb::UartPort vt03_port);
    void RxCpltCallback(uint8_t *buffer, uint16_t length);
private:
    BspUartHandle uart_ = nullptr;
    orb::UartPort vt03_port_ = orb::UartPort::U1;
    bool started_ = false;

    const float kRockerOffset = 1024.0f;
    const float kRockerNum = 660.0f;
    const float kMouseSensitivityX = 30.0f;
    const float kMouseSensitivityY = -8.0f;
    static void TaskEntry(void *param);
    void Task();

    // Internal storage to track previous raw frame
    RecviedRawData pre_uart_rx_data_{};
};