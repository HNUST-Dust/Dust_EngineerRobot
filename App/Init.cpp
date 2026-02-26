#include "Init.h"
#include "Robot.h"
#include "bsp_usart.h"

Robot robot;

void uart7_debug_callback(uint8_t *buffer, uint16_t length)
{
    robot.debug_tools_.VofaReceiveCallback(buffer, length);
}

void uart5_debug_callback(uint8_t *buffer, uint16_t length)
{
    robot.dr16_.RxCpltCallback(buffer, length);
}

void can1_callback(CanRxBuffer *CAN_RxMessage)
{
    switch (CAN_RxMessage->header.Identifier)
    {
        case (0x000):
        {
            robot.gantry_.motor_z_axis_right_.CanRxCpltCallback(CAN_RxMessage->data);
            break;
        }
        case (0x201):
        {
            robot.chassis_.motor_chassis_1_.CanRxCpltCallback(CAN_RxMessage->data);
            break;
        }
        case (0x202):
        {
            robot.chassis_.motor_chassis_2_.CanRxCpltCallback(CAN_RxMessage->data);
            break;
        }
        case (0x203):
        {
            robot.chassis_.motor_chassis_3_.CanRxCpltCallback(CAN_RxMessage->data);
            break;
        }
        case (0x204):
        {
            robot.chassis_.motor_chassis_4_.CanRxCpltCallback(CAN_RxMessage->data);
            break;
        }
        default:
            break;
    }
}

void can2_callback(CanRxBuffer *CAN_RxMessage)
{
    switch (CAN_RxMessage->header.Identifier)
    {
        case (0x00):
        {
            robot.gantry_.motor_z_axis_left_.CanRxCpltCallback(CAN_RxMessage->data);
            break;
        }
        case (0x201):
        {
            robot.gantry_.motor_x_axis_left_.CanRxCpltCallback(CAN_RxMessage->data);
            break;
        }
        case (0x202):
        {
            robot.gantry_.motor_x_axis_right_.CanRxCpltCallback(CAN_RxMessage->data);
            break;
        }

        default:
            break;
    }
}

void can3_callback(CanRxBuffer *CAN_RxMessage)
{
    switch (CAN_RxMessage->header.Identifier)
    {
        case (0x11):
        {
            robot.arm_.claws_.CanRxCpltCallback(CAN_RxMessage->data);
            break;
        }
        case (0x201):
        {
            robot.gantry_.motor_y_axis_.CanRxCpltCallback(CAN_RxMessage->data);
            break;
        }
        case (0x202):
        {
            robot.arm_.wrist_joint_left_.CanRxCpltCallback(CAN_RxMessage->data);
            break;
        }
        case (0x203):
        {
            robot.arm_.wrist_joint_right_.CanRxCpltCallback(CAN_RxMessage->data);
            break;
        }
        case (0x12):
        {
            robot.arm_.elbow_joint_yaw_.CanRxCpltCallback(CAN_RxMessage->data);
            break;
        }
        case (0x13):
        {
            robot.arm_.elbow_joint_pitch_.CanRxCpltCallback(CAN_RxMessage->data);
            break;
        }
        default:
            break;
    }
}

void Init()
{
    // UART5 初始化，DR16接收机
    uart_init(&huart5, uart5_debug_callback, UART_BUFFER_SIZE);
    // USART7 初始化，调试
    uart_init(&huart7, uart7_debug_callback, UART_BUFFER_SIZE);
    // CAN1 初始化，控制底盘 + 一个龙门架抬升电机
    can_init(&hfdcan1,can1_callback);
    // CAN2 初始化，控制龙门架 - 一个龙门架抬升电机
    can_init(&hfdcan2,can2_callback);
    // CAN3 初始化，控制机械臂
    can_init(&hfdcan3,can3_callback);

    robot.Init();
}