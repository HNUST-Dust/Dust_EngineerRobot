#include "app_arm.h"
#include "bsp_can.h"
#include "cmsis_os2.h"
#include "cmsis_os.h"
#include "dvc_motor_dji.h"
#include "dvc_motor_dm.h"
#include "fdcan.h"
#include "projdefs.h"

void Arm::Init() {

    {
        alg::PidConfig cfg;
        cfg.kp = 10.0f;
        cfg.ki = 0.8f;
        cfg.kd = 0.002f;
        cfg.kf = 0.0f;
        cfg.i_out_max = 29.0f;
        cfg.out_max = 29.0f;
        cfg.dt = 0.002f;
        claws_.pid_angle_.configure(cfg);
    }
    {
        alg::PidConfig cfg;
        cfg.kp = 0.75f;
        cfg.ki = 0.003f;
        cfg.kd = 0.001f;
        cfg.kf = 0.0f;
        cfg.i_out_max = 9.0f;
        cfg.out_max = 9.0f;
        cfg.dt = 0.002f;
        cfg.i_variable_speed_A = 0.0f;
        cfg.i_variable_speed_B = 0.0f;
        cfg.i_separate_threshold = 0.0f;
        cfg.d_first = alg::DFirst::Disable;
        cfg.d_lpf_tau = 0.02f;
        claws_.pid_omega_.configure(cfg);
    }

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


    {
        alg::PidConfig cfg;
        cfg.kp = 30.0f;
        cfg.ki = 0.15f;
        cfg.kd = 0.0f;
        cfg.kf = 1.0f;
        cfg.i_out_max = 29.0f;
        cfg.out_max = 29.0f;
        cfg.dt = 0.001f;
        wrist_joint_left_.pid_angle_.configure(cfg);
    }
    {
        alg::PidConfig cfg;
        cfg.kp = 1.0f;
        cfg.ki = 0.0f;
        cfg.kd = 0.0f;
        cfg.kf = 0.0f;
        cfg.i_out_max = 9.0f;
        cfg.out_max = 9.0f;
        cfg.dt = 0.001f;
        wrist_joint_left_.pid_omega_.configure(cfg);
    }
    wrist_joint_left_.Init(
        &hfdcan3,
        MOTOR_DJI_ID_0x202,
        MOTOR_DJI_CONTROL_METHOD_ANGLE
    );
    wrist_joint_left_.SetTargetAngle(0);


    {
        alg::PidConfig cfg;
        cfg.kp = 30.0f;
        cfg.ki = 0.15f;
        cfg.kd = 0.0f;
        cfg.kf = 1.0f;
        cfg.i_out_max = 29.0f;
        cfg.out_max = 29.0f;
        cfg.dt = 0.001f;
        wrist_joint_right_.pid_angle_.configure(cfg);
    }
    {
        alg::PidConfig cfg;
        cfg.kp = 1.0f;
        cfg.ki = 0.0f;
        cfg.kd = 0.0f;
        cfg.kf = 0.0f;
        cfg.i_out_max = 9.0f;
        cfg.out_max = 9.0f;
        cfg.dt = 0.001f;
        wrist_joint_right_.pid_omega_.configure(cfg);
    }
    wrist_joint_right_.Init(
        &hfdcan3,
        MOTOR_DJI_ID_0x203,
        MOTOR_DJI_CONTROL_METHOD_ANGLE
    );
    wrist_joint_right_.SetTargetAngle(0);


    {
        alg::PidConfig cfg;
        cfg.kp = 30.0f;
        cfg.ki = 2.8f;
        cfg.kd = 0.002f;
        cfg.kf = 0.0f;
        cfg.i_out_max = 29.0f;
        cfg.out_max = 29.0f;
        cfg.dt = 0.001f;
        elbow_joint_pitch_.pid_angle_.configure(cfg);
    }
    {
        alg::PidConfig cfg;
        cfg.kp = 3.5f;
        cfg.ki = 2.0f;
        cfg.kd = 0.0f;
        cfg.kf = 1.0f;
        cfg.i_out_max = 9.0f;
        cfg.out_max = 9.0f;
        cfg.dt = 0.001f;
        cfg.i_variable_speed_A = 0.0f;
        cfg.i_variable_speed_B = 0.0f;
        cfg.i_separate_threshold = 0.0f;
        cfg.d_first = alg::DFirst::Disable;
        cfg.d_lpf_tau = 0.02f;
        elbow_joint_pitch_.pid_omega_.configure(cfg);
    }
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


    {
        alg::PidConfig cfg;
        cfg.kp = 30.0f;
        cfg.ki = 1.8f;
        cfg.kd = 0.002f;
        cfg.kf = 0.0f;
        cfg.i_out_max = 29.0f;
        cfg.out_max = 29.0f;
        cfg.dt = 0.001f;
        elbow_joint_yaw_.pid_angle_.configure(cfg);
    }
    {
        alg::PidConfig cfg;
        cfg.kp = 1.0f;
        cfg.ki = 0.5f;
        cfg.kd = 0.0f;
        cfg.kf = 1.0f;
        cfg.i_out_max = 9.0f;
        cfg.out_max = 9.0f;
        cfg.dt = 0.001f;
        cfg.i_variable_speed_A = 0.0f;
        cfg.i_variable_speed_B = 0.0f;
        cfg.i_separate_threshold = 0.0f;
        cfg.d_first = alg::DFirst::Disable;
        cfg.d_lpf_tau = 0.02f;
        elbow_joint_yaw_.pid_omega_.configure(cfg);
    }
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
    osDelay(pdMS_TO_TICKS(100));
    claws_.CanSendEnter();
    elbow_joint_pitch_.CanSendEnter();
    elbow_joint_yaw_.CanSendEnter();
    osDelay(pdMS_TO_TICKS(1000));
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

constexpr float Clamp(float value, float min, float max) {
    if (value < min) return min;
    if (value > max) return max;
    return value;
}

void Arm::ControlClaw(float virtual_angle) {
    // 约定：virtual_angle > 0 表示“开”，virtual_angle < 0 表示“合”
    float angle = Clamp(virtual_angle, -kClawsLimit, kClawsLimit);
    claws_.SetTargetAngle(angle);
}

void Arm::ControlWristJoint(Arm::WristJointActions action, float angle) {
    switch (action) {
        case Arm::WRIST_JOINT_FLIP_UP_ACTION:
            angle = Clamp(angle, -kWristJointFlipLimit, kWristJointFlipLimit);
            wrist_joint_left_.SetTargetAngle(angle);
            wrist_joint_right_.SetTargetAngle(angle);
            break;
        case Arm::WRIST_JOINT_FLIP_DOWN_ACTION:
            angle = Clamp(angle, -kWristJointFlipLimit, kWristJointFlipLimit);
            wrist_joint_left_.SetTargetAngle(-angle);
            wrist_joint_right_.SetTargetAngle(-angle);
            break;
        case Arm::WRIST_JOINT_TWIST_LEFT_ACTION:
            angle = Clamp(angle, -kWristJointTwistLimit, kWristJointTwistLimit);
            wrist_joint_left_.SetTargetAngle(-angle);
            wrist_joint_right_.SetTargetAngle(angle);
            break;
        case Arm::WRIST_JOINT_TWIST_RIGHT_ACTION:
            angle = Clamp(angle, -kWristJointTwistLimit, kWristJointTwistLimit);
            wrist_joint_left_.SetTargetAngle(angle);
            wrist_joint_right_.SetTargetAngle(-angle);
            break;
        default:
            break;
    }
}

void Arm::ControlElbowJoint(float virtual_angle) {
    // 默认：控制 pitch 轴（翻转）。virtual_angle > 0 上翻，< 0 下翻
    float angle = Clamp(virtual_angle, -kElbowJointFlipLimit, kElbowJointFlipLimit);
    elbow_joint_pitch_.SetTargetAngle(angle);
}

void Arm::ControlElbowPitchJoint(float virtual_angle) {
    ControlElbowJoint(virtual_angle);
}

void Arm::ControlElbowYawJoint(float virtual_angle) {
    // 控制 yaw 轴（扭转）。virtual_angle > 0 右扭，< 0 左扭
    float angle = Clamp(virtual_angle, -kElbowJointTwistLimit, kElbowJointTwistLimit);
    elbow_joint_yaw_.SetTargetAngle(angle);
}

void Arm::Task() {
    for (;;) {

        claws_.CalculatePeriodElapsedCallback();

        wrist_joint_left_.CalculatePeriodElapsedCallback();
        wrist_joint_right_.CalculatePeriodElapsedCallback();
        can_send_data(&hfdcan3, 0x200, g_can3_0x200_tx_data, 8);

        elbow_joint_pitch_.CalculatePeriodElapsedCallback();
        elbow_joint_yaw_.CalculatePeriodElapsedCallback();

        osDelay(pdMS_TO_TICKS(2));// 500hz
    }
}
