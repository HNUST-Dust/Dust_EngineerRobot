#include "dr16.h"
#include "stdlib.h"
#include "string.h"

void DR16::Init()
{
    recive_raw_data_ = {
        0, 0, 0, 0, 0, 0
    };

}

void DR16::RxCpltCallback(uint8_t *rx_data, uint16_t length)
{
    recive_raw_data_.channel0 = (rx_data[0] | rx_data[1] << 8) & 0x07FF;
    recive_raw_data_.channel0 -= 1024;
    recive_raw_data_.channel1 = (rx_data[1] >> 3 | rx_data[2] << 5) & 0x07FF;
    recive_raw_data_.channel1 -= 1024;
    recive_raw_data_.channel2 = (rx_data[2] >> 6 | rx_data[3] << 2 | rx_data[4] << 10) & 0x07FF;
    recive_raw_data_.channel2 -= 1024;
    recive_raw_data_.channel3 = (rx_data[4] >> 1 | rx_data[5] << 7) & 0x07FF;
    recive_raw_data_.channel3 -= 1024;

    recive_raw_data_.switch1 = ((rx_data[5] >> 4) & 0x000C) >> 2;
    recive_raw_data_.switch2 = (rx_data[5] >> 4) & 0x0003;

    if ((abs(recive_raw_data_.channel0) > 660) || \
        (abs(recive_raw_data_.channel1) > 660) || \
        (abs(recive_raw_data_.channel2) > 660) || \
        (abs(recive_raw_data_.channel3) > 660))
    {
        memset(&recive_raw_data_, 0, sizeof(DR16::RecivedRawData));
    }

}