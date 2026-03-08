#include "app_arm.h"

#include "cmsis_os2.h"

#include <cmath>
#include <cstring>

#include "../communication_topic/topic_pubsub.hpp"
#include "../communication_topic/arm_state_topics.hpp"
#include "../communication_topic/rc_control_topics.hpp"
#include "motors/motor_instances.hpp"
#include "projdefs.h"
#include "FreeRTOS.h"
namespace {
using actuator::instances::g_claws;
using actuator::instances::g_elbow_joint_pitch;
using actuator::instances::g_elbow_joint_yaw;
using actuator::instances::g_wrist_joint_left;
using actuator::instances::g_wrist_joint_right;
}
void Arm::Init() {
    // 电机对象由 system_startup.cpp 的 startup_thread 统一实例化并注入到 Arm 单例里。

    // ---- wrist PID 配置（角度环 -> 速度目标；速度环 -> 电流）----
    {
        alg::PidConfig cfg;
        cfg.kp = 30.0f;
        cfg.ki = 0.15f;
        cfg.kd = 0.0f;
        cfg.kf = 1.0f;
        cfg.i_out_max = 29.0f;
        cfg.out_max = 29.0f;
        cfg.dt = 0.001f;
        wrist_left_pid_angle_.configure(cfg);
        wrist_right_pid_angle_.configure(cfg);
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
        wrist_left_pid_omega_.configure(cfg);
        wrist_right_pid_omega_.configure(cfg);
    }

    // ---- claws PID 配置（位置环->速度环->力矩）----
    {
        alg::PidConfig cfg;
        cfg.kp = 10.0f;
        cfg.ki = 0.8f;
        cfg.kd = 0.002f;
        cfg.kf = 0.0f;
        cfg.i_out_max = 29.0f;
        cfg.out_max = 29.0f;
        cfg.dt = 0.002f;
        claws_pid_angle_.configure(cfg);
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
        claws_pid_omega_.configure(cfg);
    }

    // ---- elbow pitch PID（位置环->速度环->力矩）----
    {
        alg::PidConfig cfg;
        cfg.kp = 30.0f;
        cfg.ki = 2.8f;
        cfg.kd = 0.002f;
        cfg.kf = 0.0f;
        cfg.i_out_max = 29.0f;
        cfg.out_max = 29.0f; // 改正
        cfg.dt = 0.001f;
        elbow_pitch_pid_angle_.configure(cfg);
    }
    {
        alg::PidConfig cfg;
        cfg.kp = 3.5f;
        cfg.ki = 2.0f;
        cfg.kd = 0.0f;
        cfg.kf = 1.0f;
        cfg.i_out_max = 9.0f;
        cfg.out_max = 9.0f; //改正
        cfg.dt = 0.001f;
        cfg.i_variable_speed_A = 0.0f;
        cfg.i_variable_speed_B = 0.0f;
        cfg.i_separate_threshold = 0.0f;
        cfg.i_separate_threshold = 0.0f;
        cfg.d_first = alg::DFirst::Disable;
        cfg.d_lpf_tau = 0.02f;
        elbow_pitch_pid_omega_.configure(cfg);
    }

    // ---- elbow yaw PID（位置环->速度环->力矩）----
    {
        alg::PidConfig cfg;
        cfg.kp = 30.0f;
        cfg.ki = 1.8f;
        cfg.kd = 0.002f;
        cfg.kf = 0.0f;
        cfg.i_out_max = 29.0f;
        cfg.out_max = 29.0f;
        cfg.dt = 0.001f;
        elbow_yaw_pid_angle_.configure(cfg);
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
        elbow_yaw_pid_omega_.configure(cfg);
    }

    // 目标角初始化
    claws_target_angle_rad_ = 0.0f;
    elbow_pitch_target_angle_rad_ = 0.0f;
    elbow_yaw_target_angle_rad_ = 0.0f;

    g_claws.ClearError();
    g_elbow_joint_pitch.ClearError();
    g_elbow_joint_yaw.ClearError();
    osDelay(100);
    g_claws.Enter();
    g_elbow_joint_pitch.Enter();
    g_elbow_joint_yaw.Enter();
    osDelay(1000);

    static const osThreadAttr_t kArmTaskAttr = {
        .name = "arm_task",
        .stack_size = 1280,
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
    claws_virtual_angle_ = Clamp(virtual_angle, -kClawsLimit, kClawsLimit);
    claws_target_angle_rad_ = claws_virtual_angle_;
}

void Arm::ControlWristByTwistFlip(float twist_target, float flip_target) {
    // left = flip + twist, right = flip - twist
    wrist_joint_left_virtual_angle_  = Clamp(flip_target + twist_target, -kWristJointTwistLimit, kWristJointTwistLimit);
    wrist_joint_right_virtual_angle_ = Clamp(flip_target - twist_target, -kWristJointTwistLimit, kWristJointTwistLimit);
}

void Arm::ControlElbowJoint(float virtual_angle) {
    elbow_pitch_joint_virtual_angle_ = Clamp(virtual_angle, -kElbowJointFlipLimit, kElbowJointFlipLimit);
    elbow_pitch_target_angle_rad_ = elbow_pitch_joint_virtual_angle_;
}

void Arm::ControlElbowYawJoint(float virtual_angle) {
    elbow_yaw_joint_virtual_angle_ = Clamp(virtual_angle, -kElbowJointTwistLimit, kElbowJointTwistLimit);
    elbow_yaw_target_angle_rad_ = elbow_yaw_joint_virtual_angle_;
}

void Arm::Task() {
    Sub<orb::RcControl> rc_sub(orb::rc_control);
    orb::RcControl rc{};

    bool last_enable = false;

    for (;;) {
        (void)rc_sub.copy(rc);

        float claws_torque = 0.0f;
        float elbow_pitch_torque = 0.0f;
        float elbow_yaw_torque = 0.0f;
        float wrist_left_current = 0.0f;
        float wrist_right_current = 0.0f;

        ControlClaw(rc.arm_claw_angle);
        ControlWristByTwistFlip(rc.arm_wrist_yaw_angle, rc.arm_wrist_pitch_angle);
        ControlElbowJoint(rc.arm_elbow_pitch_angle);
        ControlElbowYawJoint(rc.arm_elbow_yaw_angle);

        {
        // ---- DM: claws 串级 PID ----
            const float now_angle = g_claws.now_angle_rad();
            const float now_omega = g_claws.now_omega_rad_s();
            const float omega_ref = claws_pid_angle_.update(claws_target_angle_rad_, now_angle);
            claws_torque = claws_pid_omega_.update(omega_ref, now_omega);
            g_claws.SetTarget(0.0f, 0.0f, claws_torque);
            g_claws.PublishMitTx(0.0f, 0.0f);
        }
        {
        // ---- DM: elbow pitch 串级 PID ----
            const float now_angle = g_elbow_joint_pitch.now_angle_rad();
            const float now_omega = g_elbow_joint_pitch.now_omega_rad_s();
            const float omega_ref = elbow_pitch_pid_angle_.update(elbow_pitch_target_angle_rad_, now_angle);
            elbow_pitch_torque = elbow_pitch_pid_omega_.update(omega_ref, now_omega);
            g_elbow_joint_pitch.SetTarget(0.0f, 0.0f, elbow_pitch_torque);
            g_elbow_joint_pitch.PublishMitTx(0.0f, 0.0f);
        }
        {
        // ---- DM: elbow yaw 串级 PID ----
            const float now_angle = g_elbow_joint_yaw.now_angle_rad();
            const float now_omega = g_elbow_joint_yaw.now_omega_rad_s();
            const float omega_ref = elbow_yaw_pid_angle_.update(elbow_yaw_target_angle_rad_, now_angle);
            elbow_yaw_torque = elbow_yaw_pid_omega_.update(omega_ref, now_omega);
            g_elbow_joint_yaw.SetTarget(0.0f, 0.0f, elbow_yaw_torque);
            g_elbow_joint_yaw.PublishMitTx(0.0f, 0.0f);
        }
        {
        // ---- DM: wrist 串级 PID（角度环 -> 速度目标；速度环 -> 电流）----
            const float now_angle = g_wrist_joint_left.now_angle_rad();
            const float now_omega = g_wrist_joint_left.now_omega_rad_s();
            const float omega_ref = wrist_left_pid_angle_.update(wrist_joint_left_virtual_angle_, now_angle);
            wrist_left_current = wrist_left_pid_omega_.update(omega_ref, now_omega);
            g_wrist_joint_left.SetTargetCurrent(wrist_left_current);
            g_wrist_joint_left.UpdateSlot(); // 写入槽位缓冲，由 Gantry 任务统一 Flush
        }
        {
        // ---- DM: wrist 串级 PID（角度环 -> 速度目标；速度环 -> 电流）----
            const float now_angle = g_wrist_joint_right.now_angle_rad();
            const float now_omega = g_wrist_joint_right.now_omega_rad_s();
            const float omega_ref = wrist_right_pid_angle_.update(wrist_joint_right_virtual_angle_, now_angle);
            wrist_right_current = wrist_right_pid_omega_.update(omega_ref, now_omega);
            g_wrist_joint_right.SetTargetCurrent(wrist_right_current);
            g_wrist_joint_right.UpdateSlot(); // 写入槽位缓冲，由 Gantry 任务统一 Flush
        }
        g_wrist_joint_right.FlushGroup(g_wrist_joint_right); // 包含 wrist_joint_left + wrist_joint_right
        // 发布状态
        // orb::ArmState st{};
        // // st.enable = enable;
        // st.last_cmd_claws = rc.arm_claw_angle;
        // st.last_cmd_wrist_left = rc.arm_wrist_yaw_angle;
        // st.last_cmd_wrist_right = rc.arm_wrist_pitch_angle;
        // st.last_cmd_elbow_pitch = rc.arm_elbow_pitch_angle;
        // st.last_cmd_elbow_yaw = rc.arm_elbow_yaw_angle;

        // st.claws_virtual_angle = claws_virtual_angle_;
        // st.wrist_left_virtual_angle = wrist_joint_left_virtual_angle_;
        // st.wrist_right_virtual_angle = wrist_joint_right_virtual_angle_;
        // st.elbow_pitch_virtual_angle = elbow_pitch_joint_virtual_angle_;
        // st.elbow_yaw_virtual_angle = elbow_yaw_joint_virtual_angle_;

        // st.claws_now_angle = actuator::instances::g_claws.now_angle_rad();
        // st.claws_now_omega = actuator::instances::g_claws.now_omega_rad_s();

        // st.elbow_pitch_now_angle = actuator::instances::g_elbow_joint_pitch.now_angle_rad();
        // st.elbow_pitch_now_omega = actuator::instances::g_elbow_joint_pitch.now_omega_rad_s();

        // st.elbow_yaw_now_angle = actuator::instances::g_elbow_joint_yaw.now_angle_rad();
        // st.elbow_yaw_now_omega = actuator::instances::g_elbow_joint_yaw.now_omega_rad_s();

        // st.wrist_left_now_angle = actuator::instances::g_wrist_joint_left.now_angle_rad();
        // st.wrist_left_now_omega = actuator::instances::g_wrist_joint_left.now_omega_rad_s();

        // st.wrist_right_now_angle = actuator::instances::g_wrist_joint_right.now_angle_rad();
        // st.wrist_right_now_omega = actuator::instances::g_wrist_joint_right.now_omega_rad_s();

        // st.claws_torque = claws_torque;
        // st.elbow_pitch_torque = elbow_pitch_torque;
        // st.elbow_yaw_torque = elbow_yaw_torque;
        // st.wrist_left_current = wrist_left_current;
        // st.wrist_right_current = wrist_right_current;

        // orb::arm_state.publish(st);

        osDelay(pdMS_TO_TICKS(2));
    }
}
