/* Includes ------------------------------------------------------------------*/

#include "VT03.h"
#include "bsp_usart.h"

/* Private macros ------------------------------------------------------------*/
#define MOUSE_SENSITIVITY_X (30.0f)
#define MOUSE_SENSITIVITY_Y (-2.0f)
/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
static const uint16_t crc16_tab[256] =
{
    0x0000, 0x1189, 0x2312, 0x329b, 0x4624, 0x57ad, 0x6536, 0x74bf,
    0x8c48, 0x9dc1, 0xaf5a, 0xbed3, 0xca6c, 0xdbe5, 0xe97e, 0xf8f7,
    0x1081, 0x0108, 0x3393, 0x221a, 0x56a5, 0x472c, 0x75b7, 0x643e,
    0x9cc9, 0x8d40, 0xbfdb, 0xae52, 0xdaed, 0xcb64, 0xf9ff, 0xe876,
    0x2102, 0x308b, 0x0210, 0x1399, 0x6726, 0x76af, 0x4434, 0x55bd,
	0xad4a, 0xbcc3, 0x8e58, 0x9fd1, 0xeb6e, 0xfae7, 0xc87c, 0xd9f5,
	0x3183, 0x200a, 0x1291, 0x0318, 0x77a7, 0x662e, 0x54b5, 0x453c,
	0xbdcb, 0xac42, 0x9ed9, 0x8f50, 0xfbef, 0xea66, 0xd8fd, 0xc974,
	0x4204, 0x538d, 0x6116, 0x709f, 0x0420, 0x15a9, 0x2732, 0x36bb,
	0xce4c, 0xdfc5, 0xed5e, 0xfcd7, 0x8868, 0x99e1, 0xab7a, 0xbaf3,
	0x5285, 0x430c, 0x7197, 0x601e, 0x14a1, 0x0528, 0x37b3, 0x263a,
	0xdecd, 0xcf44, 0xfddf, 0xec56, 0x98e9, 0x8960, 0xbbfb, 0xaa72,
	0x6306, 0x728f, 0x4014, 0x519d, 0x2522, 0x34ab, 0x0630, 0x17b9,
	0xef4e, 0xfec7, 0xcc5c, 0xddd5, 0xa96a, 0xb8e3, 0x8a78, 0x9bf1,
	0x7387, 0x620e, 0x5095, 0x411c, 0x35a3, 0x242a, 0x16b1, 0x0738,
	0xffcf, 0xee46, 0xdcdd, 0xcd54, 0xb9eb, 0xa862, 0x9af9, 0x8b70,
	0x8408, 0x9581, 0xa71a, 0xb693, 0xc22c, 0xd3a5, 0xe13e, 0xf0b7,
	0x0840, 0x19c9, 0x2b52, 0x3adb, 0x4e64, 0x5fed, 0x6d76, 0x7cff,
	0x9489, 0x8500, 0xb79b, 0xa612, 0xd2ad, 0xc324, 0xf1bf, 0xe036,
	0x18c1, 0x0948, 0x3bd3, 0x2a5a, 0x5ee5, 0x4f6c, 0x7df7, 0x6c7e,
	0xa50a, 0xb483, 0x8618, 0x9791, 0xe32e, 0xf2a7, 0xc03c, 0xd1b5,
	0x2942, 0x38cb, 0x0a50, 0x1bd9, 0x6f66, 0x7eef, 0x4c74, 0x5dfd,
	0xb58b, 0xa402, 0x9699, 0x8710, 0xf3af, 0xe226, 0xd0bd, 0xc134,
	0x39c3, 0x284a, 0x1ad1, 0x0b58, 0x7fe7, 0x6e6e, 0x5cf5, 0x4d7c,
	0xc60c, 0xd785, 0xe51e, 0xf497, 0x8028, 0x91a1, 0xa33a, 0xb2b3,
	0x4a44, 0x5bcd, 0x6956, 0x78df, 0x0c60, 0x1de9, 0x2f72, 0x3efb,
	0xd68d, 0xc704, 0xf59f, 0xe416, 0x90a9, 0x8120, 0xb3bb, 0xa232,
	0x5ac5, 0x4b4c, 0x79d7, 0x685e, 0x1ce1, 0x0d68, 0x3ff3, 0x2e7a,
	0xe70e, 0xf687, 0xc41c, 0xd595, 0xa12a, 0xb0a3, 0x8238, 0x93b1,
	0x6b46, 0x7acf, 0x4854, 0x59dd, 0x2d62, 0x3ceb, 0x0e70, 0x1ff9,
	0xf78f, 0xe606, 0xd49d, 0xc514, 0xb1ab, 0xa022, 0x92b9, 0x8330,
	0x7bc7, 0x6a4e, 0x58d5, 0x495c, 0x3de3, 0x2c6a, 0x1ef1, 0x0f78
};

/* Private function declarations ---------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/

/**
 * @brief Get the crc16 checksum
 *
 * @param P_Msg Data to check
 * @param Length Data length
 * @param CRC16 Crc16 initialized checksum
 * @return crc16 Crc16 checksum
 */
uint16_t VT03::Get_CRC16_Check_Sum(uint8_t *P_Msg, uint16_t Length, uint16_t CRC16)
{
    uint8_t data;

    if(P_Msg == NULL)
    {
        return 0xffff;
    }

    while(Length--)
    {
        data = *P_Msg++;
        (CRC16) = ((uint16_t)(CRC16) >> 8) ^ crc16_tab[((uint16_t)(CRC16) ^ (uint16_t)(data)) & 0x00ff];
    }

    return CRC16;
}

/**
 * @brief crc16 verify function
 *
 * @param P_Msg Data to verify
 * @param len Stream length=data+checksum
 * @return bool Crc16 check result
 */
bool VT03::Verify_CRC16_Check_Sum(uint8_t *P_Msg, uint16_t Length)
{
    uint16_t w_expected = 0;

    if((P_Msg == NULL) || (Length <= 2))
    {
        return false;
    }
    w_expected = Get_CRC16_Check_Sum(P_Msg, Length - 2, CRC16_Init);

    return ((w_expected & 0xff) == P_Msg[Length - 2] && ((w_expected >> 8) & 0xff) == P_Msg[Length - 1]);
}

/**
 * @brief VT03初始化
 *
 * @param huart 指定的UART
 */
void VT03::Init(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        UART_Manage_Object = &g_uart1_manage_object;
    }
    else if (huart->Instance == USART2)
    {
        UART_Manage_Object = &g_uart2_manage_object;
    }
    else if (huart->Instance == USART3)
    {
        UART_Manage_Object = &g_uart3_manage_object;
    }
    else if (huart->Instance == UART4)
    {
        UART_Manage_Object = &g_uart4_manage_object;
    }
    else if (huart->Instance == UART5)
    {
        UART_Manage_Object = &g_uart5_manage_object;
    }
    else if (huart->Instance == USART6)
    {
        UART_Manage_Object = &g_uart6_manage_object;
    }
    else if (huart->Instance == UART7)
    {
        UART_Manage_Object = &g_uart7_manage_object;
    }
    else if (huart->Instance == UART8)
    {
        UART_Manage_Object = &g_uart8_manage_object;
    }
}

/**
 * @brief UART通信接收回调函数
 *
 * @param Rx_Data 接收的数据
 */
void VT03::UART_RxCpltCallback(uint8_t *Rx_Data, uint16_t Length)
{
    // 滑动窗口, 判断遥控器VT03是否在线
    Flag += 1;

    Data_Process(Length);
}

/**
 * @brief 定时器计算函数
 *
 */
void VT03::TIM_1ms_Calculate_PeriodElapsedCallback()
{
    // 数据处理过程
    RemoteData *tmp_buffer = (RemoteData *) UART_Manage_Object->rx_buffer;

    // 判断拨码触发
    _Judge_Switch(&Data.Mode_Switch, tmp_buffer->Mode_Switch, Pre_UART_Rx_Data.Mode_Switch);

    // 判断鼠标触发
    _Judge_Key(&Data.Mouse_Left_Key, tmp_buffer->Mouse_Left_Key, Pre_UART_Rx_Data.Mouse_Left_Key);
    _Judge_Key(&Data.Mouse_Right_Key, tmp_buffer->Mouse_Right_Key, Pre_UART_Rx_Data.Mouse_Right_Key);

    // 判断键盘触发
    for (int i = 0; i < 16; i++)
    {
        _Judge_Key(&Data.Keyboard_Key[i], ((tmp_buffer->Key) >> i) & 0x1, ((Pre_UART_Rx_Data.Key) >> i) & 0x1);
    }

    // 保留数据
    memcpy(&Pre_UART_Rx_Data, tmp_buffer, 21 * sizeof(uint8_t));
}

/**
 * @brief 数据处理过程
 *
 */
void VT03::Data_Process(uint16_t Length)
{
    // 数据处理过程
    RemoteData *tmp_buffer = (RemoteData *) UART_Manage_Object->rx_buffer;
    if((tmp_buffer->Start_Of_Frame_1 == 0xA9) && (tmp_buffer->Start_Of_Frame_2 == 0x53)){
        // 摇杆信息
        Data.Right_X = (tmp_buffer->Channel_0 - Rocker_Offset) / Rocker_Num; //右摇杆水平
        if(Data.Right_X > 0.99f){
            Data.Right_X = 0.99f;
        }else if(Data.Right_X < -0.99f){
            Data.Right_X = -0.99f;
        }
        Data.Right_Y = (tmp_buffer->Channel_1 - Rocker_Offset) / Rocker_Num; // 右摇杆竖直
        if(Data.Right_X > 0.99f){
            Data.Right_X = 0.99f;
        }else if(Data.Right_X < -0.99f){
            Data.Right_X = -0.99f;
        }
        Data.Left_X = (tmp_buffer->Channel_2 - Rocker_Offset) / Rocker_Num;  // 左摇杆竖直
        Data.Left_Y = (tmp_buffer->Channel_3 - Rocker_Offset) / Rocker_Num;  // 左摇杆水平

        // 自定义按键信息
        _Judge_Key(&Data.Left_Key, tmp_buffer->Fn_1, Pre_UART_Rx_Data.Fn_1);
        _Judge_Key(&Data.Right_Key, tmp_buffer->Fn_2, Pre_UART_Rx_Data.Fn_2);
        _Judge_Key(&Data.Trigger, tmp_buffer->Trigger, Pre_UART_Rx_Data.Trigger);
        _Judge_Key(&Data.Pause, tmp_buffer->Pause, Pre_UART_Rx_Data.Pause);

        // 拨轮信息
        Data.Wheel = (tmp_buffer->Wheel - Rocker_Offset) / Rocker_Num;

        // 档位拨动开关信息
        switch(tmp_buffer->Mode_Switch)
        {
            case 0:
            Data.Mode_Switch = VT03_STATUS_LEFT;
            break;
            case 1:
            Data.Mode_Switch = VT03_STATUS_MIDDLE;
            break;
            case 2:
            Data.Mode_Switch = VT03_STATUS_RIGHT;
            break;
            default:
            Data.Mode_Switch = VT03_STATUS_MIDDLE;
            break;
        }

        // 鼠标信息
        Data.Mouse_X = tmp_buffer->Mouse_X / 32768.0f * MOUSE_SENSITIVITY_X;
        Data.Mouse_Y += tmp_buffer->Mouse_Y / 32768.0f * MOUSE_SENSITIVITY_Y;
        if(Data.Mouse_Y > 0.5f){
            Data.Mouse_Y = 0.5f;
        }else if(Data.Mouse_Y < -0.5f){
            Data.Mouse_Y = -0.5f;
        }
        Data.Mouse_Z = tmp_buffer->Mouse_Z / 32768.0f;
        _Judge_Key(&Data.Mouse_Left_Key, tmp_buffer->Mouse_Left_Key, Pre_UART_Rx_Data.Mouse_Left_Key);
        _Judge_Key(&Data.Mouse_Right_Key, tmp_buffer->Mouse_Right_Key, Pre_UART_Rx_Data.Mouse_Right_Key);
        _Judge_Key(&Data.Mouse_Middle_Key, tmp_buffer->Mouse_Middle_Key, Pre_UART_Rx_Data.Mouse_Middle_Key);
        // 键盘信息
        for (int i = 0; i < 16; i++)
        {
            _Judge_Key(&Data.Keyboard_Key[i], ((tmp_buffer->Key) >> i) & 0x1, ((Pre_UART_Rx_Data.Key) >> i) & 0x1);
        }

        // 保留数据
        memcpy(&Pre_UART_Rx_Data, tmp_buffer, 21 * sizeof(uint8_t));
    }else{
        // Data.Right_X = 0.0f;
        // Data.Right_Y = 0.0f;
        // Data.Left_X = 0.0f;
        // Data.Left_Y = 0.0f;
        // Data.Mode_Switch = VT03_STATUS_MIDDLE;
        // Data.Mouse_X = 0.0f;
        // Data.Mouse_Y = 0.0f;
        // Data.Mouse_Z = 0.0f;
        // Data.Left_Key = VT03_Key_Status_FREE;
        // Data.Right_Key = VT03_Key_Status_FREE;
        // Data.Pause = VT03_Key_Status_FREE;
        // Data.Trigger = VT03_Key_Status_FREE;
        // Data.Mouse_Left_Key = VT03_Key_Status_FREE;
        // Data.Mouse_Right_Key = VT03_Key_Status_FREE;
        // Data.Keyboard_Key[0] = VT03_Key_Status_FREE;
        // Data.Keyboard_Key[1] = VT03_Key_Status_FREE;
        // Data.Keyboard_Key[2] = VT03_Key_Status_FREE;
        // Data.Keyboard_Key[3] = VT03_Key_Status_FREE;
        // Data.Keyboard_Key[4] = VT03_Key_Status_FREE;
        // Data.Keyboard_Key[5] = VT03_Key_Status_FREE;
        // Data.Keyboard_Key[6] = VT03_Key_Status_FREE;
        // Data.Keyboard_Key[7] = VT03_Key_Status_FREE;
        // Data.Keyboard_Key[8] = VT03_Key_Status_FREE;
        // Data.Keyboard_Key[9] = VT03_Key_Status_FREE;
        // Data.Keyboard_Key[10] = VT03_Key_Status_FREE;
        // Data.Keyboard_Key[11] = VT03_Key_Status_FREE;
        // Data.Keyboard_Key[12] = VT03_Key_Status_FREE;
        // Data.Keyboard_Key[13] = VT03_Key_Status_FREE;
        // Data.Keyboard_Key[14] = VT03_Key_Status_FREE;
        // Data.Keyboard_Key[15] = VT03_Key_Status_FREE;
        // Data.Wheel = 127;
    }
}

/**
 * @brief 判断按键状态
 *
 */
void VT03::_Judge_Key(VT03KeyStatus *Key, uint8_t Status, uint8_t Pre_Status)
{
    // 带触发的判断
    switch (Pre_Status)
    {
    case (VT03_KEY_FREE):
    {
        switch (Status)
        {
        case (VT03_KEY_FREE):
        {
            *Key = VT03_Key_Status_FREE;

            break;
        }
        case (VT03_KEY_PRESSED):
        {
            *Key = VT03_Key_Status_TRIG_FREE_PRESSED;

            break;
        }
        }

        break;
    }
    case (VT03_KEY_PRESSED):
    {
        switch (Status)
        {
        case (VT03_KEY_FREE):
        {
            *Key = VT03_Key_Status_TRIG_PRESSED_FREE;

            break;
        }
        case (VT03_KEY_PRESSED):
        {
            *Key = VT03_Key_Status_PRESSED;

            break;
        }
        }

        break;
    }
    }
}

/************************ COPYRIGHT(C) HNUST-DUST **************************/
