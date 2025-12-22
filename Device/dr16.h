#ifndef DEVICE_DR16_H
#define DEVICE_DR16_H

#include <cstdint>
class DR16
{
public:
    enum SwitchStatus {
        UP   = (uint8_t)1,
        DOWN = (uint8_t)2,
        MID  = (uint8_t)3,
    };

    struct RecivedRawData {
        int16_t channel0;
        int16_t channel1;
        int16_t channel2;
        int16_t channel3;
        uint8_t switch1;
        uint8_t switch2;
    } __attribute__((packed));

    struct RecivedProcessedData {

    };

    void Init();
    RecivedRawData* GetRawData() {
        return &recive_raw_data_;
    }
    void RxCpltCallback(uint8_t *rx_data, uint16_t length);

private:
    RecivedRawData recive_raw_data_;
    RecivedProcessedData recived_processed_data_;
};

#endif
