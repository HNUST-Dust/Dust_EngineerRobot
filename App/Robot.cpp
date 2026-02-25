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
    float claws_virtual_angle = 0.0f;
    float wrist_left_joint_virtual_angle = 0.0f;
    float wrist_right_joint_virtual_angle = 0.0f;
    float elbow_yaw_joint_virtual_angle = 0.0f;
    float elbow_pitch_joint_virtual_angle = 0.0f;
    float gantry_z_virtual_distance = 0.0f;

    float twist = 0.f; // 正为向右扭转
    float flip = 0.f; // 正为向上翻转
    float left_target = 0.f;
    float right_target = 0.f;
    for (;;) {
        switch (dr16_.GetData()->left_switch) {
            case 1:
                // 底盘模式
                chassis_.SetTargetVxInChassis(+ dr16_.GetData()->left_stick_x * CHASSIS_SPEED);
                chassis_.SetTargetVyInChassis(- dr16_.GetData()->left_stick_y * CHASSIS_SPEED);
                chassis_.SetTargetVelocityRotation(dr16_.GetData()->wheel* CHASSIS_SPEED);        
                break;
            case 2:
                // 手臂模式
                
                arm_.claws_virtual_angle_ -= dr16_.GetData()->right_stick_x * arm_.CLAWS_SENSITIVITY;
                if(arm_.claws_virtual_angle_ < 0) {
                    arm_.ControlClaw(Arm::CLAW_CLOSE_ACTION, arm_.claws_virtual_angle_);
                } else {
                    arm_.ControlClaw(Arm::CLAW_OPEN_ACTION, -arm_.claws_virtual_angle_);
                }

                twist = dr16_.GetData()->wheel * arm_.WRIST_SENSITIVITY; // 正为向右扭转
                flip = dr16_.GetData()->right_stick_y * arm_.WRIST_SENSITIVITY; // 正为向上翻转
                // left = flip + twist, right = flip - twist）
                arm_.wrist_joint_left_virtual_angle_ += flip + twist;
                arm_.wrist_joint_right_virtual_angle_ += flip - twist;
                arm_.wrist_joint_left_.SetTargetAngle(arm_.wrist_joint_left_virtual_angle_);
                arm_.wrist_joint_right_.SetTargetAngle(arm_.wrist_joint_right_virtual_angle_);


                arm_.elbow_pitch_joint_virtual_angle_ += dr16_.GetData()->left_stick_y * arm_.ELBOW_PITCH_SENSITIVITY;
                if (arm_.elbow_pitch_joint_virtual_angle_ > 0) {
                    arm_.ControlElbowJoint(Arm::ELBOW_JOINT_FLIP_UP_ACTION, arm_.elbow_pitch_joint_virtual_angle_);
                } else {
                    arm_.ControlElbowJoint(Arm::ELBOW_JOINT_FLIP_DOWN_ACTION, -arm_.elbow_pitch_joint_virtual_angle_);
                }
                arm_.elbow_yaw_joint_virtual_angle_ += dr16_.GetData()->left_stick_x * arm_.ELBOW_YAW_SENSITIVITY;
                if (arm_.elbow_yaw_joint_virtual_angle_ > 0) {
                    arm_.ControlElbowJoint(Arm::ELBOW_JOINT_TWIST_RIGHT_ACTION, arm_.elbow_yaw_joint_virtual_angle_);
                } else {
                    arm_.ControlElbowJoint(Arm::ELBOW_JOINT_TWIST_LEFT_ACTION, -arm_.elbow_yaw_joint_virtual_angle_);
                }
                break;
            case 3:
                // 龙门架模式
                gantry_.XAxisMoveInSpeed(dr16_.GetData()->left_stick_y * 10.f); 
                gantry_.YAxisMoveInSpeed(dr16_.GetData()->left_stick_x * 10.f);
                gantry_.ZAxisMoveInSpeed(dr16_.GetData()->right_stick_y *10.f);
                // gantry_.XAxisMoveInDistance(10.f);
                // gantry_.YAxisMoveInDistance(10.f);
                // gantry_.ZAxisMoveInDistance(10.f);
        
                break;
            default:
                break;
        }
        /********************** 底盘 ***********************/ 
        
        /********************** 测试用例 ***********************/ 


        /********************** 调试信息 ***********************/
        debug_tools_.VofaSendFloat(arm_.elbow_pitch_joint_virtual_angle_);
        debug_tools_.VofaSendFloat(arm_.elbow_joint_pitch_.GetNowAngleNoncumulative());
        debug_tools_.VofaSendFloat(dr16_.GetData()->right_stick_y);
        // // 调试帧尾部
        debug_tools_.VofaSendTail();

        osDelay(pdMS_TO_TICKS(1));// 1khz

    }
}