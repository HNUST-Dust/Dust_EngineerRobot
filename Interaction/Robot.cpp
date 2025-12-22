/**
 * @file Robot.cpp
 * @author noe (noneofever@gmail.com)
 * @brief 
 * @version 0.1
 * @date 2025-08-04
 * 
 * @copyright Copyright (c) 2025
 * 
 */
#include "Robot.h"
#include "cmsis_os2.h"

// app
#include "app_chassis.h"
#include "alg_math.h"
// module
#include "dvc_MCU_comm.h"
#include "debug_tools.h"

// bsp
#include "bsp_dwt.h"

// FreeRTOS stack check
#include "task.h"


void Robot::Init()
{
    dwt_init(480);
    debug_tools_.VofaInit();
    // 陀螺仪初始化
    // imu_.Init();
    osDelay(pdMS_TO_TICKS(10000));// 10s时间等待陀螺仪收敛

    // 底盘初始化
    chassis_.Init();

    static const osThreadAttr_t kRobotTaskAttr = {
        .name = "robot_task",
        .stack_size = 768,
        .priority = (osPriority_t) osPriorityNormal
    };
    osThreadNew(Robot::TaskEntry, this, &kRobotTaskAttr);
}

void Robot::TaskEntry(void *argument)
{
    Robot *self = static_cast<Robot *>(argument);
    self->Task();
}

void Robot::Task()
{

    for (;;)
    {
        __disable_irq();
        __enable_irq();

        /********************** 底盘 ***********************/ 
        // chassis_.SetTargetVxInGimbal((mcu_comm_data_local.chassis_speed_x - 127.0f) * CHASSIS_SPEED / 128.0f); //9
        // chassis_.SetTargetVyInGimbal((127.0f - mcu_comm_data_local.chassis_speed_y ) * CHASSIS_SPEED / 128.0f); //9
        // chassis_.SetTargetVelocityRotation(((127.0f - mcu_comm_data_local.chassis_rotation ) * CHASSIS_SPEED / 128.0f));
        // chassis_.SetYawAngle(-normalize_angle_pm_pi(gimbal_.GetNowYawAngle()/YAW_GEAR_RATIO));
    
        /********************** 模式切换 ***********************/   
        // switch(mcu_comm_data_local.chassis_spin)
        // {
        //     case CHASSIS_SPIN_CLOCKWISE:
        //         chassis_.SetTargetVelocityRotation(CHASSIS_SPIN_SPEED);
        //         gimbal_.SetGimbalYawControlType(GIMBAL_CONTROL_TYPE_OMEGA);
        //         gimbal_.SetYawOmegaFeedforword(YAW_FEEDFORWORD_RATIO * CHASSIS_SPIN_SPEED);
        //         gimbal_.SetTargetYawOmega((mcu_comm_data_local.yaw - 127.0f)*YAW_SPEED_SENSITIVITY); //补偿速度可能符号错了
        //     break;
        //     case CHASSIS_SPIN_DISABLE:
        //         chassis_.SetTargetVelocityRotation(0.0f);
        //         gimbal_.SetGimbalYawControlType(GIMBAL_CONTROL_TYPE_ANGLE);
        //         gimbal_.SetYawOmegaFeedforword(0.0f);
        //     break;
        //     case CHASSIS_SPIN_COUNTER_CLOCK_WISE: // 疯车保护
        //         chassis_.Exit();
        //         gimbal_.Exit();
        //     break;
        //     default:
        //     // do nothing
        //     break; 
        // };
 
        /********************** 调试信息 ***********************/   
        // debug_tools_.VofaSendFloat(mcu_comm_.mcu_imu_data_.yaw_total_angle_f); 
        // debug_tools_.VofaSendFloat(mcu_comm_.mcu_autoaim_data_.yaw_angle * RAD_TO_DEG);
        // debug_tools_.VofaSendFloat(virtual_yaw_angle_);
        // // 调试帧尾部
        // debug_tools_.VofaSendTail();

        osDelay(pdMS_TO_TICKS(1));// 1khz
    }
}
