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
    // chassis_.Init();
    // 手臂初始化
    arm_.Init();
    // 龙门架初始化
    // gantry_.Init();

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
    float wrist_joint_virtual_angle = 0.0f;
    float elbow_joint_virtual_angle = 0.0f;
    float gantry_x_virtual_distance = 0.0f;
    float gantry_y_virtual_distance = 0.0f;
    float gantry_z_virtual_distance = 0.0f;

    for (;;)
    {
        __disable_irq();
        __enable_irq();

        /********************** 底盘 ***********************/ 
        
        /********************** 测试用例 ***********************/ 
        // chassis_.SetTargetVxInChassis(+ dr16_.GetData()->left_stick_x * CHASSIS_SPEED);
        // chassis_.SetTargetVyInChassis(- dr16_.GetData()->left_stick_y * CHASSIS_SPEED);
        // chassis_.SetTargetVelocityRotation(dr16_.GetData()->wheel* CHASSIS_SPEED);

        gantry_.XAxisMoveInSpeed(dr16_.GetData()->left_stick_x * 10.f); 
        gantry_.YAxisMoveInSpeed(dr16_.GetData()->left_stick_y * 10.f);
        gantry_.ZAxisMoveInSpeed(dr16_.GetData()->right_stick_y *10.f);

        // gantry_.XAxisMoveInDistance(10.f);
        // gantry_.YAxisMoveInDistance(10.f);
        // gantry_.ZAxisMoveInDistance(10.f);

        // arm_.ControlClaw(Arm::CLAW_CLOSE_ACTION, 1.f);
        // arm_.ControlClaw(Arm::CLAW_OPEN_ACTION, 0.0f);
        // arm_.ControlWristJoint(Arm::WRIST_JOINT_FLIP_UP_ACTION, 1.f);
        // arm_.ControlWristJoint(Arm::WRIST_JOINT_FLIP_DOWN_ACTION, 1.f);
        // arm_.ControlElbowJoint(Arm::ELBOW_JOINT_FLIP_UP_ACTION, 1.f);
        // arm_.ControlElbowJoint(Arm::ELBOW_JOINT_FLIP_DOWN_ACTION, 1.f);
    
        /********************** 调试信息 ***********************/
        debug_tools_.VofaSendFloat(static_cast<float>(dr16_.GetData()->left_stick_y)); // 开关1
        // // 调试帧尾部
        debug_tools_.VofaSendTail();

        osDelay(pdMS_TO_TICKS(1));// 1khz
    }
}
