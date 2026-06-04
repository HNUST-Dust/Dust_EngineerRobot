#pragma once
#include "dvc_motor_dji.h"
#include "dvc_motor_dm.h"

class Arm {
public:
    MotorDmNormal claws_;

    MotorDmNormal elbow_joint_yaw_;
    MotorDmNormal elbow_joint_pitch_;

    // 手腕关节
    MotorDjiC610 wrist_joint_left_;
    MotorDjiC610 wrist_joint_right_;

    constexpr static float kClawsSensitivity = 0.001f;
    float claws_virtual_angle_ = 0.0f;

    constexpr static float kWristSensitivity = 0.003f;
    float wrist_joint_left_virtual_angle_ = 0.0f;
    float wrist_joint_right_virtual_angle_ = 0.0f;

    constexpr static float kElbowYawSensitivity = 0.001f;
    constexpr static float kElbowPitchSensitivity = 0.001f;
    float elbow_yaw_joint_virtual_angle_ = 0.0f;
    float elbow_pitch_joint_virtual_angle_ = 0.0f;

    void Init();
    void Task();

    // 手爪：只传虚拟角度，正负表示开/合方向
    void ControlClaw(float virtual_angle);

    // 肘关节：统一只传虚拟角度
    // 说明：此接口默认控制pitch（翻转）轴；yaw请使用ControlElbowYawJoint
    void ControlElbowJoint(float virtual_angle);

    void ControlElbowPitchJoint(float virtual_angle);
    void ControlElbowYawJoint(float virtual_angle);

    // 手腕：输入 twist/flip 的增量角度，内部完成左右联动并下发目标角
    // 约定：flip > 0 为向上翻转；twist > 0 为向右扭转
    void ControlWristByTwistFlip(float twist_delta, float flip_delta);

private:
    static constexpr float kClawsLimit = 0.35f;

    static constexpr float kWristJointFlipLimit = 1.0f;
    static constexpr float kWristJointTwistLimit = 1.0f;

    static constexpr float kElbowJointFlipLimit = 1.0f;
    static constexpr float kElbowJointTwistLimit = 3.14f;

    static void TaskEntry(void *param);
};