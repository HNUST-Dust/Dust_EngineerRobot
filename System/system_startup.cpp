/**
 * @file system_startup.cpp
 * @brief 工程统一启动序列实现（System_Boot + staged bring-up）
 *
 * 核心逻辑：
 * =========
 * - `System_Boot()` 按固定顺序执行：BSP -> Board -> Modules -> App。
 * - 将“模块拉起顺序”集中管理，避免分散在各处导致依赖关系难以审查。
 *
 * 关键依赖：
 * =========
 * - DWT 时间基在 Board_BringUp() 早期初始化，供 daemon/控制环路使用。
 * - daemon_supervisor 必须在各模块注册 client 前 init。
 * - CAN/UART 发送统一经由 TxTask：这里负责启动 `CanTxTask`/`UartTxTask` 作为唯一发送出口。
 *
 * 故障策略：
 * =========
 * - 通过 `DaemonSupervisor::set_system_fault_hook()` 设置系统级故障回调。
 *   当前实现为 best-effort：请求底盘/云台退出，并发布执行器层的管理指令。
 */

#include "system_startup.h"
#include "AppWiring.h"
#include "app_arm.h"
#include "app_chassis.h"
#include "app_gantry.h"

#include "bsp_can_port.h"
#include "bsp_uart_port.h"
#include "bsp_dwt.h"
#include "../daemon_supervisor/supervisor.hpp"
#include "../Drivers/can_tx_task.h"
#include "../Drivers/uart_tx_task.h"
#include "../Drivers/spi_tx_task.h"
#include "../Communication/dvc_mcu_comm.h"
#include "../Device/debug_tools.h"
#include "../Device/motor_ids.hpp"
#include "motors/dji_c6xx.hpp"
#include "dvc_vt03.h"

#include "bmi088.h"

namespace {
static void daemon_system_fault(DaemonClient&)
{

}
} // namespace

void Bsp_BringUp(void)
{
    dwt_init(480);
    
    App_WirePlatformIo();

    static CanTxTask s_can_tx_task;
    s_can_tx_task.Start();

    static UartTxTask s_uart_tx_task;
    s_uart_tx_task.Start();

    static SpiTxTask s_spi_tx_task;
    s_spi_tx_task.Start();

    // USB serialization/send is handled by BSP driver (bsp_usb_send_pc)
}

void Board_BringUp(void)
{
    // Board-level devices on PCB (LEDs, IMU power/reset, etc.)
}

void Modules_BringUp(void)
{
    // 守护系统初始化
    DaemonSupervisor::Start(daemon_system_fault);

    // BMI088 模块：采样 + 解算 + 发布 orb::imu_data
    static Bmi088 s_bmi088;
    s_bmi088.Start();

    // 大疆电机初始化（CAN1）    
    actuator::drivers::DjiC6xxMin::Config c{};
    c.bus = orb::CanBus::CAN1;
    c.method = actuator::drivers::DjiC6xxMin::ControlMethod::Omega;

    c.gearbox_ratio = 1.0f;
    c.kp = 0.0f;
    c.ki = 0.0f;
    c.kd = 0.0f;
    c.current_limit = 20.0f;

    auto c1 = c;
    c1.rx_std_id = static_cast<uint16_t>(motor_ids::kLeftWheel);
    auto c2 = c;
    c2.rx_std_id = static_cast<uint16_t>(motor_ids::kRightWheel);
    auto c3 = c;
    c3.rx_std_id = static_cast<uint16_t>(motor_ids::kPoke);

    auto* can1 = bsp_can_get(BSP_CAN_BUS1);
    configASSERT(can1 != nullptr);
    
    actuator::instances::dji_201.Init(can1, c1);
    actuator::instances::dji_201.JoinRuntime();

    actuator::instances::dji_202.Init(can1, c2);
    actuator::instances::dji_202.JoinRuntime();

    actuator::instances::dji_203.Init(can1, c3);
    actuator::instances::dji_203.JoinRuntime();

    // 上下板通讯组件初始化（CAN2）
    McuComm::Instance().Init(orb::CanBus::CAN2,bsp_can_get(BSP_CAN_BUS2));

    // // 裁判系统初始化（UART1 RX already started in Bsp_BringUp）
    // Referee::Instance().Init(bsp_uart_get(BSP_UART1), orb::UartPort::U1);

    // 接收机初始化 
    VT03::Instance().Init(bsp_uart_get(BSP_UART1), orb::UartPort::U1);

    // VOFA 初始化（UART7 RX already started in Bsp_BringUp）
    DebugTools::Instance().Init(bsp_uart_get(BSP_UART7), orb::UartPort::U7);
}

void App_Start(void)
{
    Booster::Instance().Init();
}

void startup_thread(void *argument)
{
    Bsp_BringUp();
    Board_BringUp();
    Modules_BringUp();
    App_Start();
    vTaskDelete(NULL);
}