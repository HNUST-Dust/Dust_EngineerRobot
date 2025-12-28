#include "app_arm.h"
#include "bsp_can.h"
#include "cmsis_os2.h"
#include "cmsis_os.h"
#include "dvc_motor_dji.h"
#include "dvc_motor_dm.h"
#include "fdcan.h"

void Arm::Init() {

    claws_.pid_angle_.Init(
        10.0f,
        0.8f,
        0.002f,
        0.0f,
        29.0f,
        29.0f,
        0.001f
    );
    claws_.pid_omega_.Init(
        0.75f,
        0.003f,
        0.001f,
        0.0f,
        9.0f,
        9.0f,
        0.001f,
        0.0f,
        0.0f,
        0.0f,
        0.0f,
        PID_D_First_DISABLE,
        0.02f
    );
    claws_.Init(
        &hfdcan3,
        0x11,
        0x01,
        MOTOR_DM_CONTROL_METHOD_NORMAL_MIT,
        ANGLE_OMEGA_CLOSE_LOOP_MODE,
        12.5f,
        29.0f,
        10.0f
    );
    claws_.SetKp(0);
    claws_.SetKd(0.f);
    claws_.SetControlAngle(0);
    claws_.SetControlOmega(0);
    claws_.SetControlTorque(0);


    wrist_joint_left_.pid_angle_.Init(
        30.0f,
        0.15f,
        0.0f,
        1.0f,
        29.0f,
        29.0f,
        0.001f
    );
    wrist_joint_left_.pid_omega_.Init(
        1.0f,
        0.0f,
        0.0f,
        0.0f,
        9.0f,
        9.0f,
        0.001f
    );
    wrist_joint_left_.Init(
        &hfdcan3,
        MOTOR_DJI_ID_0x202,
        MOTOR_DJI_CONTROL_METHOD_ANGLE
    );
    wrist_joint_left_.SetTargetAngle(0);


    wrist_joint_right_.pid_angle_.Init(
        30.0f,
        0.15f,
        0.0f,
        1.0f,
        29.0f,
        29.0f,
        0.001f
    );
    wrist_joint_right_.pid_omega_.Init(
        1.0f,
        0.0f,
        0.0f,
        0.0f,
        9.0f,
        9.0f,
        0.001f
    );
    wrist_joint_right_.Init(
        &hfdcan3,
        MOTOR_DJI_ID_0x203,
        MOTOR_DJI_CONTROL_METHOD_ANGLE
    );
    wrist_joint_right_.SetTargetAngle(0);

    elbow_joint_pitch_.pid_angle_.Init(
        1.0f,
        0.0f,
        0.0f,
        0.0f,
        29.0f,
        29.0f,
        0.001f
    );
    elbow_joint_pitch_.pid_omega_.Init(
        1.0f,
        0.0f,
        0.0f,
        0.0f,
        9.0f,
        9.0f,
        0.001f
    );
    elbow_joint_pitch_.Init(
        &hfdcan3,
        0x13,
        0x03,
        MOTOR_DM_CONTROL_METHOD_NORMAL_MIT,
        ANGLE_OMEGA_CLOSE_LOOP_MODE,
        12.5f,
        10.0f,
        28.0f
    );
    elbow_joint_pitch_.SetKp(0);
    elbow_joint_pitch_.SetKd(0.f);
    elbow_joint_pitch_.SetControlAngle(0);
    elbow_joint_pitch_.SetControlOmega(0);
    elbow_joint_pitch_.SetControlTorque(0);


    elbow_joint_yaw_.pid_angle_.Init(
        1.0f,
        0.0f,
        0.0f,
        0.0f,
        29.0f,
        29.0f,
        0.001f
    );
    elbow_joint_yaw_.pid_omega_.Init(
        1.0f,
        0.0f,
        0.0f,
        0.0f,
        9.0f,
        9.0f,
        0.001f
    );
    elbow_joint_yaw_.Init(
        &hfdcan3,
        0x12,
        0x02,
        MOTOR_DM_CONTROL_METHOD_NORMAL_MIT,
        ANGLE_OMEGA_CLOSE_LOOP_MODE,
        12.5f,
        29.0f,
        10.0f
    );
    elbow_joint_yaw_.SetKp(0);
    elbow_joint_yaw_.SetKd(0.f);
    elbow_joint_yaw_.SetControlAngle(0);
    elbow_joint_yaw_.SetControlOmega(0);
    elbow_joint_yaw_.SetControlTorque(0);


    claws_.CanSendClearError();
    elbow_joint_pitch_.CanSendClearError();
    elbow_joint_yaw_.CanSendClearError();
    osDelay(100);
    claws_.CanSendEnter();
    elbow_joint_pitch_.CanSendEnter();
    elbow_joint_yaw_.CanSendEnter();
    osDelay(1000);
    claws_.Output();
    elbow_joint_pitch_.Output();
    elbow_joint_yaw_.Output();

    static const osThreadAttr_t kArmTaskAttr = {
        .name = "arm_task",
        .stack_size = 1024,
        .priority = (osPriority_t) osPriorityNormal
    };
    osThreadNew(Arm::TaskEntry, this, &kArmTaskAttr);
}

void Arm::TaskEntry(void *param) {
    Arm *arm = static_cast<Arm *>(param);
    arm->Task();
}

void Arm::ControlClaw(Arm::ClawActions action, float angle) {
    switch (action) {
        case Arm::CLAW_OPEN_ACTION:
            claws_.SetControlAngle(angle);
            break;
        case Arm::CLAW_CLOSE_ACTION:
            claws_.SetControlAngle(-angle);
            break;
        default:
            break;
    }
}

void Arm::ControlWristJoint(Arm::WristJointActions action, float angle) {
    switch (action) {
        case Arm::WRIST_JOINT_FLIP_UP_ACTION:
            wrist_joint_left_.SetTargetAngle(angle);
            wrist_joint_right_.SetTargetAngle(angle);
            break;
        case Arm::WRIST_JOINT_FLIP_DOWN_ACTION:
            wrist_joint_left_.SetTargetAngle(-angle);
            wrist_joint_right_.SetTargetAngle(-angle);
            break;
        case Arm::WRIST_JOINT_TWIST_LEFT_ACTION:
            wrist_joint_left_.SetTargetAngle(-angle);
            wrist_joint_right_.SetTargetAngle(angle);
            break;
        case Arm::WRIST_JOINT_TWIST_RIGHT_ACTION:
            wrist_joint_left_.SetTargetAngle(angle);
            wrist_joint_right_.SetTargetAngle(-angle);
            break;
        default:
            break;
    }
}

void Arm::ControlElbowJoint(Arm::ElbowJointActions action, float angle) {
    switch (action) {
        case Arm::ELBOW_JOINT_FLIP_UP_ACTION:
            elbow_joint_pitch_.SetControlAngle(angle);
            break;
        case Arm::ELBOW_JOINT_FLIP_DOWN_ACTION:
            elbow_joint_pitch_.SetControlAngle(-angle);
            break;
        case Arm::ELBOW_JOINT_TWIST_LEFT_ACTION:
            elbow_joint_yaw_.SetControlAngle(-angle);
            break;
        case Arm::ELBOW_JOINT_TWIST_RIGHT_ACTION:
            elbow_joint_yaw_.SetControlAngle(angle);
            break;
        default:
            break;
    }
}

void Arm::Task() {
    for (;;) {

        claws_.CalculatePeriodElapsedCallback();
        
        wrist_joint_left_.CalculatePeriodElapsedCallback();
        wrist_joint_right_.CalculatePeriodElapsedCallback();
        can_send_data(&hfdcan3, 0x200, g_can3_0x200_tx_data, 8);

        elbow_joint_pitch_.CalculatePeriodElapsedCallback();
        elbow_joint_yaw_.CalculatePeriodElapsedCallback();

        osDelay(pdMS_TO_TICKS(1));// 1khz
    }
}
