#include "dr16.h"
#include "stdlib.h"
#include "string.h"

void DR16::Init()
{
    recive_raw_data_ = {
        0, 0, 0, 0, 0, 0, 0
    };

}

void DR16::RxCpltCallback(uint8_t *rx_data, uint16_t length)
{
    // 解析通道
    recive_raw_data_.channel0 = (int16_t)(((int16_t)rx_data[0] | (int16_t)rx_data[1] << 8) & 0x07FF) - 1024;
    recive_raw_data_.channel1 = (int16_t)(((int16_t)rx_data[1] >> 3 | (int16_t)rx_data[2] << 5) & 0x07FF) - 1024;
    recive_raw_data_.channel2 = (int16_t)(((int16_t)rx_data[2] >> 6 | (int16_t)rx_data[3] << 2 | (int16_t)rx_data[4] << 10) & 0x07FF) - 1024;
    recive_raw_data_.channel3 = (int16_t)(((int16_t)rx_data[4] >> 1 | (int16_t)rx_data[5] << 7) & 0x07FF) - 1024;

    recive_raw_data_.switch1 = ((rx_data[5] >> 4) & 0x000C) >> 2;
    recive_raw_data_.switch2 = (rx_data[5] >> 4) & 0x0003;
	recive_raw_data_.pulley_wheel = -(int16_t)(((((int16_t)rx_data[16]) | ((int16_t)rx_data[17]<<8)) & 0x07FF) - 1024);

    // 数据异常处理
    if ((abs(recive_raw_data_.channel0) > 660) || \
    (abs(recive_raw_data_.channel1) > 660) || \
    (abs(recive_raw_data_.channel2) > 660) || \
    (abs(recive_raw_data_.channel3) > 660))
    {
        memset(&recive_raw_data_, 0, sizeof(DR16::RecivedRawData));
    }

    // 归一化处理
    recived_processed_data_.right_stick_x = recive_raw_data_.channel0 / 660.0f;
    recived_processed_data_.right_stick_y = recive_raw_data_.channel1 / 660.0f;
    recived_processed_data_.left_stick_x  = recive_raw_data_.channel2 / 660.0f;
    recived_processed_data_.left_stick_y  = recive_raw_data_.channel3 / 660.0f;
    recived_processed_data_.left_switch  = recive_raw_data_.switch1;
    recived_processed_data_.right_switch = recive_raw_data_.switch2;
    recived_processed_data_.wheel = recive_raw_data_.pulley_wheel / 660.0f;
}