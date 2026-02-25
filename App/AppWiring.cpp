/**
 * @file AppWiring.cpp
 * @brief 平台 IO 装配实现：UART/CAN RX → 对应模块回调
 *
 * 核心逻辑：
 * =========
 * - 通过 `bsp_uart_init()` 注册 UART RX 回调。
 * - 通过 `bsp_can_add_rx_callback()` 注册 CAN RX 回调（支持多个订阅者扇出）。
 * - 回调内部只做“ID 分发/转发”，将数据交给模块实例处理。
 *
 * 数据流（以当前实现为准）：
 * ========================
 * - UART7: Debug/VOFA → `DebugTools::Instance().VofaReceiveCallback()`
 * - UART1: Referee → `Referee::Instance().RxCpltCallback()`
 * - CAN1/2/3: Motors → 按电机 ID 分发到各电机实例 `CanRxCpltCallback()`
 * - CAN2(特定 ID): 外部 MCU 数据 → `McuComm::Instance().RxCpltCallback()`
 * - CAN3(0x100): Supercap → `Supercap::Instance().CanRxCpltCallback()`
 *
 * 注意事项：
 * =========
 * - 该文件不做 CAN/UART 发送；发送必须走统一的 TxTask/Topic 出口。
 * - 回调可能在中断上下文执行：必须避免阻塞与复杂计算。
 */

#include "AppWiring.h"

#include "bsp_can_port.h"
#include "bsp_uart_port.h"
#include "bsp_usb_port.h"

#include "../Communication/dvc_pc_comm.h"
#include "../Device/debug_tools.h"

#include "../Device/motor_ids.hpp"

#include "motors/dji_c6xx.hpp"
#include "motors/dm_mit.hpp"
#include "motors/cubemars_mit.hpp"
#include "dvc_dr16.h"

// USART7 VOFA debug
static void uart7_debug_callback(uint8_t *buffer, uint16_t length)
{
    DebugTools::Instance().VofaReceiveCallback(buffer, length);
}

// USART5 接收机
static void uart5_referee_callback(uint8_t *buffer, uint16_t length)
{
    Dr16::Instance().RxCpltCallback(buffer, length);
}

// 底盘电机 + 龙门Z轴右电机
static void can1_rx_callback(const BspCanFrame* frame)
{
    if (!frame) {
        return;
    }
    if (frame->id_type != BSP_CAN_ID_STD || frame->frame_type != BSP_CAN_FRAME_DATA || frame->len < 8u) {
        return;
    }

    switch (frame->id) {
    case motor_ids::kMotorZAxisRight:
        actuator::instances::motor_z_axis_right.CanRxCpltCallback(frame);
        break;
    case motor_ids::kMotorChassis1:
        actuator::instances::chassis_wheel_1.CanRxCpltCallback(frame);
        break;
    case motor_ids::kMotorChassis2:
        actuator::instances::chassis_wheel_2.CanRxCpltCallback(frame);
        break;
    case motor_ids::kMotorChassis3:
        actuator::instances::chassis_wheel_3.CanRxCpltCallback(frame);
        break;
    case motor_ids::kMotorChassis4:
        actuator::instances::chassis_wheel_4.CanRxCpltCallback(frame);
        break;
    default:
        break;
    }
}

// X 轴 + 龙门Z轴左电机
static void can2_rx_callback(const BspCanFrame* frame)
{
    if (!frame) {
        return;
    }
    if (frame->id_type != BSP_CAN_ID_STD || frame->frame_type != BSP_CAN_FRAME_DATA || frame->len < 8u) {
        return;
    }
    switch (frame->id) {
    case motor_ids::kMotorZAxisLeft:
        actuator::instances::motor_z_axis_left.CanRxCpltCallback(frame);
        break;
    case motor_ids::kMotorXAxisRight:
        actuator::instances::motor_x_axis_right.CanRxCpltCallback(frame);
        break;
    case motor_ids::kMotorXAxisLeft:
        actuator::instances::motor_x_axis_left.CanRxCpltCallback(frame);
        break;
    default:
        break;
    }
}

// 手臂 + 龙门Y轴
static void can3_rx_callback(const BspCanFrame* frame)
{
    if (!frame) {
        return;
    }
    if (frame->id_type != BSP_CAN_ID_STD || frame->frame_type != BSP_CAN_FRAME_DATA || frame->len < 8u) {
        return;
    }
    switch (frame->id) {
    case motor_ids::kMotorElbowPitch:
        actuator::instances::elbow_joint_pitch.CanRxCpltCallback(frame);
        break;
    case motor_ids::kMotorElbowYaw:
        actuator::instances::elbow_joint_yaw.CanRxCpltCallback(frame);
        break;
    case motor_ids::kMotorWristLeft:
        actuator::instances::wrist_joint_left.CanRxCpltCallback(frame);
        break;
    case motor_ids::kMotorWristRight:
        actuator::instances::wrist_joint_right.CanRxCpltCallback(frame);
        break;
    case motor_ids::kMotorClaws:
        actuator::instances::claws.CanRxCpltCallback(frame);
        break;
    case motor_ids::kMotorYAxis:
        actuator::instances::gantry_motor_y_axis.CanRxCpltCallback(frame);
        break;
    default:
        break;
    }
}

static void usb_tx_callback(uint16_t len)
{
    // 目前不需要处理发送完成事件
}

static void usb_rx_callback(uint16_t len)
{
    PcComm::Instance().RxCpltCallback(len);
}

void App_WirePlatformIo(void)
{
    constexpr uint16_t kUartRxBufferSize = 512;

    bsp_usb_init(bsp_usb_get(), usb_tx_callback, usb_rx_callback);

    // UART
    bsp_uart_init(bsp_uart_get(BSP_UART7), uart7_debug_callback, kUartRxBufferSize);
    bsp_uart_init(bsp_uart_get(BSP_UART5), uart5_referee_callback, kUartRxBufferSize);

    // CAN
    auto* can1 = bsp_can_get(BSP_CAN_BUS1);
    auto* can2 = bsp_can_get(BSP_CAN_BUS2);

    (void)bsp_can_add_rx_callback(can1, can1_rx_callback);
    (void)bsp_can_add_rx_callback(can2, can2_rx_callback);
}
