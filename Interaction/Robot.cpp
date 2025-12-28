/**
 * @file Robot.cpp
 * @author noe (noneofever@gmail.com)
 * @brief 
 * @version 0.1
 * @date 2025-12-22
 * 
 * @copyright Copyright (c) 2025
 * 
 */
// app
#include "Robot.h"
#include "app_chassis.h"
#include "app_arm.h"
#include "cmsis_os2.h"
// module
#include "debug_tools.h"

// bsp
#include "bsp_dwt.h"
#include "projdefs.h"


void Robot::Init()
{
    osDelay(pdMS_TO_TICKS(1000));
    
    dwt_init(480);
    debug_tools_.VofaInit();

    // dr16初始化
    dr16_.Init();
    // 底盘初始化
    chassis_.Init();
    // 手臂初始化
    arm_.Init();
    // 龙门架初始化
    gantry_.Init();

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
        // chassis_.SetTargetVxInChassis((dr16_.GetRawData()->channel0 - 660.0f) * CHASSIS_SPEED / 660.0f); //9
        // chassis_.SetTargetVyInChassis((660.0f - dr16_.GetRawData()->channel1) * CHASSIS_SPEED / 660.0f); //9
        // chassis_.SetTargetVelocityRotation(((127.0f - dr16_.GetRawData()->channel2) * CHASSIS_SPEED / 128.0f));

        /********************** 测试用例 ***********************/ 
        // chassis_.SetTargetVxInChassis(0);
        // chassis_.SetTargetVyInChassis(0);

        // gantry_.XAxisMove(10); 
        // gantry_.YAxisMove(10);
        // gantry_.ZAxisMoveInSpeed(10);
        // gantry_.XAxisMoveInSpeed(1.0f);

        arm_.ControlClaw(Arm::CLAW_OPEN_ACTION, 1.0f);
        /********************** 调试信息 ***********************/
        debug_tools_.VofaSendFloat(static_cast<float>(dr16_.GetRawData()->channel0));
        // // 调试帧尾部
        debug_tools_.VofaSendTail();

        osDelay(pdMS_TO_TICKS(1));// 1khz
    }
}
