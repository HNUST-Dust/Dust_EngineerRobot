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
#include "app_gantry.h"
#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "task.h"

// module
#include "debug_tools.h"

// bsp
#include "bsp_dwt.h"

// algorithm
#include "math/alg_math.h"
#include <cmath>
#include "utils/alg_constrain.h"

void Robot::Init()
{
    osDelay(pdMS_TO_TICKS(1000));
    
    dwt_init(480);
    debug_tools_.VofaInit();

    // dr16初始化
    dr16_.Init();
    vt03_.Init(&huart10);
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
    float twist = 0.f; // 正为向右扭转
    float flip = 0.f; // 正为向上翻转
    for (;;) {
        if (dr16_.GetData()->right_switch == 1) {
            switch (dr16_.GetData()->left_switch) {
                case 1:
                    // 底盘模式
                    chassis_.SetTargetVxInChassis(+ dr16_.GetData()->left_stick_x * kChassisSpeed);
                    chassis_.SetTargetVyInChassis(- dr16_.GetData()->left_stick_y * kChassisSpeed);
                    chassis_.SetTargetVelocityRotation(dr16_.GetData()->wheel* kChassisSpeed);
                    break;
                case 2:
                    // 手臂模式
                    arm_.claws_virtual_angle_ -= dr16_.GetData()->right_stick_x * arm_.kClawsSensitivity;
                    math_constrain(&arm_.claws_virtual_angle_, -0.5f, 0.5f);
                    arm_.ControlClaw(arm_.claws_virtual_angle_);

                    twist = dr16_.GetData()->wheel * arm_.kWristSensitivity; // 正为向右扭转
                    flip = dr16_.GetData()->right_stick_y * arm_.kWristSensitivity; // 正为向上翻转
                    arm_.ControlWristByTwistFlip(twist, flip);

                    arm_.elbow_pitch_joint_virtual_angle_ += dr16_.GetData()->left_stick_y * arm_.kElbowPitchSensitivity;
                    arm_.ControlElbowJoint(arm_.elbow_pitch_joint_virtual_angle_);

                    arm_.elbow_yaw_joint_virtual_angle_ += dr16_.GetData()->left_stick_x * arm_.kElbowYawSensitivity;
                    arm_.ControlElbowYawJoint(arm_.elbow_yaw_joint_virtual_angle_);
                    break;
                case 3:
                    // 龙门架模式
                    gantry_.XAxisMoveInSpeed(dr16_.GetData()->left_stick_y * 10.f); 
                    gantry_.YAxisMoveInSpeed(dr16_.GetData()->left_stick_x * 10.f);
                    // gantry_.ZAxisMoveInSpeed(dr16_.GetData()->right_stick_y * 10.f);
                    gantry_.virtual_z_distance_ -= dr16_.GetData()->right_stick_y * gantry_.Z_AXIS_SENSITIVITY;
                    math_constrain(&gantry_.virtual_z_distance_, -gantry_.Z_AXIS_DISTANCE_LIMIT, 0.0f);

                    gantry_.ZAxisMoveInDistance(gantry_.virtual_z_distance_);
            
                    break;
                default:
                    break;
            }
        } else if (dr16_.GetData()->right_switch == 0 || dr16_.GetData()->right_switch == 3) {
            static constexpr uint8_t kButton1Mask = 1u << 0;
            static constexpr uint8_t kButton2Mask = 1u << 1;
            static constexpr uint8_t kButton3Mask = 1u << 2;
            static constexpr uint8_t kButton4Mask = 1u << 3;
            static constexpr uint8_t kJoystickButtonMask = 1u << 4;
            static constexpr float kClawToggleAngle = 0.5f;
            static constexpr float kClawSlewRatePerSec = 1.5f;
            static constexpr float kLoopPeriodSec = 0.001f;

            static bool claw_closed = false;
            static bool claw_state_inited = false;
            static bool prev_joystick_pressed = false;
            static float claw_cmd_angle = 0.0f;
            static float claw_target_angle = 0.0f;

            const bool joystick_pressed = (vt03_.ControllerData.buttons & kJoystickButtonMask) != 0;
            if (!claw_state_inited) {
                claw_closed = (arm_.claws_virtual_angle_ < 0.0f);
                claw_target_angle = claw_closed ? -kClawToggleAngle : +kClawToggleAngle;
                claw_cmd_angle = claw_target_angle;
                claw_state_inited = true;
            }
            if (joystick_pressed && !prev_joystick_pressed) {
                claw_closed = !claw_closed;
            }
            prev_joystick_pressed = joystick_pressed;

            claw_target_angle = claw_closed ? -kClawToggleAngle : +kClawToggleAngle;
            const float max_step = kClawSlewRatePerSec * kLoopPeriodSec;
            const float delta = claw_target_angle - claw_cmd_angle;
            if (std::fabs(delta) <= max_step) {
                claw_cmd_angle = claw_target_angle;
            } else {
                claw_cmd_angle += (delta > 0.0f) ? max_step : -max_step;
            }

            const uint8_t buttons = static_cast<uint8_t>(vt03_.ControllerData.buttons & (kButton1Mask | kButton2Mask | kButton3Mask | kButton4Mask | kJoystickButtonMask));
            switch (buttons) {
                case kButton1Mask:
                    // BUTTON1 单独按下
                    chassis_.SetTargetVxInChassis(- vt03_.ControllerData.joystick_x * kChassisSpeed);
                    chassis_.SetTargetVyInChassis(+ vt03_.ControllerData.joystick_y * kChassisSpeed);
                    chassis_.SetTargetVelocityRotation(+ vt03_.ControllerData.joystick_z * kChassisSpeed);
                    break;
                case kButton2Mask:
                    // BUTTON2 单独按下
                    gantry_.XAxisMoveInSpeed(-vt03_.ControllerData.joystick_y * 5.0f);
                    gantry_.YAxisMoveInSpeed(-vt03_.ControllerData.joystick_x * 5.0f);
                    gantry_.virtual_z_distance_ -= vt03_.ControllerData.joystick_z * 0.008f;
                    math_constrain(&gantry_.virtual_z_distance_, -gantry_.Z_AXIS_DISTANCE_LIMIT, 0.0f);
                    gantry_.ZAxisMoveInDistance(gantry_.virtual_z_distance_);                    
                    break;
                case kButton3Mask:
                    // BUTTON3 单独按下
                    twist = vt03_.ControllerData.joystick_y * arm_.kWristSensitivity; // 正为向右扭转
                    flip = vt03_.ControllerData.joystick_z * arm_.kWristSensitivity; // 正为向上翻转
                    arm_.ControlWristByTwistFlip(twist, flip);

                    break;
                case kButton4Mask:
                    // BUTTON4 单独按下
                    break;
                default:
                    chassis_.SetTargetVxInChassis(0.0f);
                    chassis_.SetTargetVyInChassis(0.0f);
                    chassis_.SetTargetVelocityRotation(0.0f);
                    gantry_.XAxisMoveInSpeed(0.0f);
                    gantry_.YAxisMoveInSpeed(0.0f);

                    break;
            }

            arm_.claws_virtual_angle_ = claw_cmd_angle;
            math_constrain(&arm_.claws_virtual_angle_, -0.5f, 0.5f);
            arm_.ControlClaw(arm_.claws_virtual_angle_);

            arm_.elbow_pitch_joint_virtual_angle_ = -vt03_.ControllerData.angle1;
            arm_.ControlElbowPitchJoint(arm_.elbow_pitch_joint_virtual_angle_);

            arm_.elbow_yaw_joint_virtual_angle_ = vt03_.ControllerData.angle3;
            arm_.ControlElbowYawJoint(arm_.elbow_yaw_joint_virtual_angle_);
        }

        /********************** 调试信息 ***********************/
        debug_tools_.VofaSendFloat(vt03_.ControllerData.angle1);
        debug_tools_.VofaSendFloat(vt03_.ControllerData.angle2);
        debug_tools_.VofaSendFloat(vt03_.ControllerData.angle3);
        debug_tools_.VofaSendFloat(vt03_.ControllerData.angle4);
        debug_tools_.VofaSendFloat(vt03_.ControllerData.joystick_x);
        debug_tools_.VofaSendFloat(vt03_.ControllerData.joystick_y);
        debug_tools_.VofaSendFloat(vt03_.ControllerData.joystick_z);
        debug_tools_.VofaSendFloat((float)vt03_.ControllerData.buttons);
        debug_tools_.VofaSendFloat(gantry_.virtual_z_distance_);
        // 调试帧尾部
        debug_tools_.VofaSendTail();

        osDelay(pdMS_TO_TICKS(1));// 1khz
    }
}