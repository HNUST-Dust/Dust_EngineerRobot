#pragma once
#include "dvc_motor_dji.h"
#include "dvc_motor_dm.h"

class Arm {
public:
    // 手爪
    enum ClawActions {
        CLAW_OPEN_ACTION = 0,
        CLAW_CLOSE_ACTION,
    };
    MotorDmNormal claws_;

    // 手腕关节
    enum WristJointActions {
        WRIST_JOINT_FLIP_UP_ACTION = 0,
        WRIST_JOINT_FLIP_DOWN_ACTION,
        WRIST_JOINT_TWIST_LEFT_ACTION,
        WRIST_JOINT_TWIST_RIGHT_ACTION,
    };
    MotorDjiC610 wrist_joint_left_;
    MotorDjiC610 wrist_joint_right_;

    // 肘部关节
    enum ElbowJointActions {
        ELBOW_JOINT_FLIP_UP_ACTION = 0,
        ELBOW_JOINT_FLIP_DOWN_ACTION,
        ELBOW_JOINT_TWIST_LEFT_ACTION,
        ELBOW_JOINT_TWIST_RIGHT_ACTION,
    };
    MotorDmNormal elbow_joint_yaw_;
    MotorDmNormal elbow_joint_pitch_;

    void Init();
    void Task();
    void ControlClaw(ClawActions action, float angle);
    void ControlWristJoint(WristJointActions action, float angle);
    void ControlElbowJoint(ElbowJointActions action, float angle);

private:
    static constexpr float CLAWS_LIMIT = 1.0f;
    static constexpr float WRIST_JOINT_FLIP_LIMIT = 1.0f;
    static constexpr float WRIST_JOINT_TWIST_LIMIT = 1.0f;
    static constexpr float ELBOW_JOINT_FLIP_LIMIT = 1.0f;
    static constexpr float ELBOW_JOINT_TWIST_LIMIT = 1.0f;

    static void TaskEntry(void *param);
};