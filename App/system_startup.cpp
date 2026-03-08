#include "system_startup.h"

#include "app_chassis.h"
#include "app_arm.h"
#include "app_gantry.h"

#include "cmsis_os2.h"

#include "bsp_can_port.h"
#include "bsp_uart_port.h"

#include "bsp_dwt.h"

#include "debug_tools.h"
#include "dvc_dr16.h"
#include "dvc_vt03.h"
#include "dvc_pc_comm.h"
#include "../Device/motors/motor_instances.hpp"

// 通过引用别名统一访问全局电机实例，避免潜在的宏/解析冲突
namespace {
using actuator::instances::g_claws;
using actuator::instances::g_elbow_joint_pitch;
using actuator::instances::g_elbow_joint_yaw;

using actuator::instances::g_motor_chassis_1;
using actuator::instances::g_motor_chassis_2;
using actuator::instances::g_motor_chassis_3;
using actuator::instances::g_motor_chassis_4;

using actuator::instances::g_motor_x_axis_left;
using actuator::instances::g_motor_x_axis_right;
using actuator::instances::g_motor_y_axis;

using actuator::instances::g_wrist_joint_left;
using actuator::instances::g_wrist_joint_right;

using actuator::instances::g_motor_z_axis_left;
using actuator::instances::g_motor_z_axis_right;
}

namespace {
constexpr uint16_t kUartRxBufferSize = 512;

// 电机对象在启动线程中统一实例化并绑定 CAN，App 模块只负责控制逻辑
BspCanHandle s_can1 = nullptr;
BspCanHandle s_can2 = nullptr;
BspCanHandle s_can3 = nullptr;
}

void uart7_debug_callback(uint8_t *buffer, uint16_t length)
{
    DebugTools::Instance().VofaReceiveCallback(buffer, length);
}

void uart5_debug_callback(uint8_t *buffer, uint16_t length)
{
    Dr16::Instance().RxCpltCallback(buffer, length);
}

void uart10_debug_callback(uint8_t *buffer, uint16_t length)
{
    //VT03::Instance().RxCpltCallback(buffer, length);
}

static void can1_port_callback(const BspCanFrame* frame)
{
    if (!frame) return;

    switch (frame->id)
    {
        case 0x000:
            g_motor_z_axis_right.CanRxCpltCallback(frame);
            break;
        case 0x201:
            g_motor_chassis_1.CanRxCpltCallback(frame);
            break;
        case 0x202:
            g_motor_chassis_2.CanRxCpltCallback(frame);
            break;
        case 0x203:
            g_motor_chassis_3.CanRxCpltCallback(frame);
            break;
        case 0x204:
            g_motor_chassis_4.CanRxCpltCallback(frame);
            break;
        default:
            break;
    }
}

static void can2_port_callback(const BspCanFrame* frame)
{
    if (!frame) return;

    switch (frame->id)
    {
        case 0x000:
            g_motor_z_axis_left.CanRxCpltCallback(frame);
            break;
        case 0x201:
            g_motor_x_axis_left.CanRxCpltCallback(frame);
            break;
        case 0x202:
            g_motor_x_axis_right.CanRxCpltCallback(frame);
            break;
        default:
            break;
    }
}

static void can3_port_callback(const BspCanFrame* frame)
{
    if (!frame) return;

    switch (frame->id)
    {
        case 0x11:
            g_claws.CanRxCpltCallback(frame);
            break;
        case 0x201:
            g_motor_y_axis.CanRxCpltCallback(frame);
            break;
        case 0x202:
            g_wrist_joint_left.CanRxCpltCallback(frame);
            break;
        case 0x203:
            g_wrist_joint_right.CanRxCpltCallback(frame);
            break;
        case 0x12:
            g_elbow_joint_yaw.CanRxCpltCallback(frame);
            break;
        case 0x13:
            g_elbow_joint_pitch.CanRxCpltCallback(frame);
            break;
        default:
            break;
    }
}

void bsp_bringup(void)
{
    dwt_init(480);

    // UART 初始化：用 bsp_uart_port
    bsp_uart_init(bsp_uart_get(BSP_UART5), uart5_debug_callback, kUartRxBufferSize);
    bsp_uart_init(bsp_uart_get(BSP_UART7), uart7_debug_callback, kUartRxBufferSize);
    bsp_uart_init(bsp_uart_get(BSP_USART10), uart10_debug_callback, kUartRxBufferSize);

    // 绑定 CAN 句柄（集中管理，后续 App 层不再调用 bsp_can_get）
    s_can1 = bsp_can_get(BSP_CAN_BUS1);
    s_can2 = bsp_can_get(BSP_CAN_BUS2);
    s_can3 = bsp_can_get(BSP_CAN_BUS3);

    // CAN 初始化：用 bsp_can_port（支持多回调；这里每个 bus 注册一个分发器）
    bsp_can_add_rx_callback(s_can1, can1_port_callback);
    bsp_can_add_rx_callback(s_can2, can2_port_callback);
    bsp_can_add_rx_callback(s_can3, can3_port_callback);
}

void board_bringup(void)
{


}

void modules_bringup(void)
{
    DebugTools::Instance().Init(bsp_uart_get(BSP_UART7));
    DebugTools::Instance().StartThread();

    Dr16::Instance().Init(bsp_uart_get(BSP_UART5));
   // VT03::Instance().Init(bsp_uart_get(BSP_USART10));

    // 先实例化协议层电机，并绑定到模块对象上（便于 CAN 回调分发）

    // ---- Chassis motors (CAN1) ----
    {
        actuator::drivers::DjiC6xxMin::Config cfg{};
        cfg.bus = 1;
        cfg.tx_std_id = 0x200;
        cfg.gearbox_ratio = 268.0f / 17.0f;
        cfg.current_limit = 20.0f;
        cfg.enc_per_round = 8192;

        cfg.rx_std_id = 0x201;
        g_motor_chassis_1.Init(s_can1, cfg);
        cfg.rx_std_id = 0x202;
        g_motor_chassis_2.Init(s_can1, cfg);
        cfg.rx_std_id = 0x203;
        g_motor_chassis_3.Init(s_can1, cfg);
        cfg.rx_std_id = 0x204;
        g_motor_chassis_4.Init(s_can1, cfg);
    }

    // ---- Gantry motors (CAN1/CAN2/CAN3) ----
    // Z-left  -> CAN2, rx=0x00 tx=0x01
    // Z-right -> CAN1, rx=0x00 tx=0x01
    // X-left/right -> CAN2, rx=0x201/0x202, group tx=0x200
    // Y -> CAN3, rx=0x201, group tx=0x200
    {
        actuator::drivers::CubemarsMitMin::Config cfg;
        cfg.bus = 2;
        cfg.rx_std_id = 0x000;
        cfg.tx_std_id = 0x001;
        cfg.angle_max = 12.5f;
        cfg.omega_max = 50.0f;
        cfg.torque_max = 65.0f;
        cfg.enter_after_control = true;
        ::actuator::instances::g_motor_z_axis_left.Init(s_can2, cfg);

        cfg.bus = 1;
        ::actuator::instances::g_motor_z_axis_right.Init(s_can1, cfg);
    }
    {
        actuator::drivers::DjiC6xxMin::Config cfg;
        cfg.bus = 2;
        cfg.rx_std_id = 0x201;
        cfg.tx_std_id = 0x200;
        cfg.gearbox_ratio = 268.0f / 17.0f;
        cfg.current_limit = 20.0f;
        ::actuator::instances::g_motor_x_axis_left.Init(s_can2, cfg);

        cfg.rx_std_id = 0x202;
        ::actuator::instances::g_motor_x_axis_right.Init(s_can2, cfg);

        cfg.bus = 3;
        cfg.rx_std_id = 0x201;
        cfg.tx_std_id = 0x200;
        cfg.gearbox_ratio = 36.0f;
        cfg.current_limit = 10.0f;
        cfg.current_to_raw = 10000.0f / 10.0f; // C610(M2006)
        ::actuator::instances::g_motor_y_axis.Init(s_can3, cfg);
    }


    // ---- Arm motors (CAN3) ----
    {
        // claws
        actuator::drivers::DmMitMin::Config cfg;
        cfg.bus = 3;
        cfg.master_id = 0x11;
        cfg.can_rx_id = 0x01;
        cfg.angle_max = 12.5f;
        cfg.omega_max = 29.0f;
        cfg.torque_max = 10.0f;
        ::actuator::instances::g_claws.Init(s_can3, cfg);
    }
    {
        actuator::drivers::DmMitMin::Config cfg;
        cfg.bus = 3;
        cfg.master_id = 0x13;
        cfg.can_rx_id = 0x03;
        cfg.angle_max = 12.5f;
        cfg.omega_max = 10.0f;
        cfg.torque_max = 28.0f;
        ::actuator::instances::g_elbow_joint_pitch.Init(s_can3, cfg);
    }
    {
        actuator::drivers::DmMitMin::Config cfg;
        cfg.bus = 3;
        cfg.master_id = 0x12;
        cfg.can_rx_id = 0x02;
        cfg.angle_max = 12.5f;
        cfg.omega_max = 45.0f;
        cfg.torque_max = 10.0f;
        ::actuator::instances::g_elbow_joint_yaw.Init(s_can3, cfg);
    }
    {
        actuator::drivers::DjiC6xxMin::Config cfg;
        cfg.bus = 3;
        cfg.rx_std_id = 0x202;
        cfg.tx_std_id = 0x200;
        cfg.gearbox_ratio = 36.0f;
        cfg.current_limit = 10.0f;
        cfg.current_to_raw = 10000.0f / 10.0f; // C610(M2006)
        ::actuator::instances::g_wrist_joint_left.Init(s_can3, cfg);

        cfg.rx_std_id = 0x203;
        ::actuator::instances::g_wrist_joint_right.Init(s_can3, cfg);
    }
}

void app_bringup(void)
{
    Chassis::Instance().Init();
    Arm::Instance().Init();
    Gantry::Instance().Init();
}
void startup_thread(void *argument)
{
    osDelay(1000);
    bsp_bringup();
    board_bringup();
    modules_bringup();
    app_bringup();

    osThreadExit();
}