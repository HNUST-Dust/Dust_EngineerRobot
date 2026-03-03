#pragma once

#include <memory>

#include "../Algorithm/control/alg_pid.h"

#include "../Device/motors/dji_c6xx.hpp"
#include "../Device/motors/dm_mit.hpp"

class Arm {
public:
    static Arm& Instance()
    {
        static Arm instance;
        return instance;
    }

    // 纯协议层电机（上层持有，禁止默认构造)

    // 手腕 PID（上层角度控制 -> 输出电流）
    alg::Pid wrist_left_pid_angle_;
    alg::Pid wrist_left_pid_omega_;
    alg::Pid wrist_right_pid_angle_;
    alg::Pid wrist_right_pid_omega_;

    // ---- 外置串级 PID（位置环->速度环->输出）----
    // claws
    alg::Pid claws_pid_angle_;
    alg::Pid claws_pid_omega_;

    // elbow pitch
    alg::Pid elbow_pitch_pid_angle_;
    alg::Pid elbow_pitch_pid_omega_;

    // elbow yaw
    alg::Pid elbow_yaw_pid_angle_;
    alg::Pid elbow_yaw_pid_omega_;

    // ---- 目标量（单位：rad / rad/s）----
    float claws_target_angle_rad_ = 0.0f;
    float elbow_pitch_target_angle_rad_ = 0.0f;
    float elbow_yaw_target_angle_rad_ = 0.0f;

    constexpr static float kClawsSensitivity = 0.001f;
    float claws_virtual_angle_ = 0.0f;

    constexpr static float kWristSensitivity = 0.002f;
    float wrist_joint_left_virtual_angle_ = 0.0f;
    float wrist_joint_right_virtual_angle_ = 0.0f;

    constexpr static float kElbowYawSensitivity = 0.001f;
    constexpr static float kElbowPitchSensitivity = 0.001f;
    float elbow_yaw_joint_virtual_angle_ = 0.0f;
    float elbow_pitch_joint_virtual_angle_ = 0.0f;

    void Init();
    void Task();

    void ControlClaw(float virtual_angle);
    void ControlElbowJoint(float virtual_angle);

    void ControlElbowPitchJoint(float virtual_angle);
    void ControlElbowYawJoint(float virtual_angle);

    void ControlWristByTwistFlip(float twist_delta, float flip_delta);

private:
    Arm() = default;
    Arm(const Arm&) = delete;
    Arm& operator=(const Arm&) = delete;

    static constexpr float kClawsLimit = 0.35f;

    static constexpr float kWristJointFlipLimit = 1.0f;
    static constexpr float kWristJointTwistLimit = 1.0f;

    static constexpr float kElbowJointFlipLimit = 1.0f;
    static constexpr float kElbowJointTwistLimit = 3.14f;

    static void TaskEntry(void *param);
};

// 兼容层（可逐步移除）：保留旧函数名，内部转发到 Instance()
inline Arm& ArmInstance() { return Arm::Instance(); }