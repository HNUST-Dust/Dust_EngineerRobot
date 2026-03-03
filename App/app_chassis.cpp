// app
#include "app_chassis.h"

#include "../communication_topic/chassis_topics.hpp"
#include "../communication_topic/chassis_cmd_topics.hpp"
#include "../communication_topic/chassis_state_topics.hpp"
#include "../communication_topic/topic_pubsub.hpp"
#include "../communication_topic/rc_control_topics.hpp"

#include "../Device/motors/motor_instances.hpp"

namespace {
using actuator::instances::g_motor_chassis_1;
using actuator::instances::g_motor_chassis_2;
using actuator::instances::g_motor_chassis_3;
using actuator::instances::g_motor_chassis_4;

constexpr float kChassisSpeed = 10.0f;
}

void Chassis::Init()
{
    // 电机对象由 system_startup.cpp 的 startup_thread 统一实例化并注入到 Chassis 单例里。

    // ---- 底盘四电机速度环 PID（rad/s -> A）----
    // 说明：这里只给默认参数，占位可跑；需要你后续根据实际车体与电机调参。
    // 输出单位为电流(A)，最终会在 DjiC6xxMin 内部限幅到 cfg.current_limit。
    alg::PidConfig omega_cfg{};
    omega_cfg.kp = 1.0f;
    omega_cfg.ki = 0.0f;
    omega_cfg.kd = 0.0f;
    omega_cfg.kf = 0.0f;
    omega_cfg.dt = 0.001f;    // 1kHz
    omega_cfg.out_max = 20.0f; // A（与 cfg.current_limit 对齐）
    omega_cfg.i_out_max = 10.0f;

    pid_omega_1_.configure(omega_cfg);
    pid_omega_2_.configure(omega_cfg);
    pid_omega_3_.configure(omega_cfg);
    pid_omega_4_.configure(omega_cfg);
    pid_omega_1_.reset();
    pid_omega_2_.reset();
    pid_omega_3_.reset();
    pid_omega_4_.reset();

    g_motor_chassis_1.SetTargetCurrent(0.0f);
    g_motor_chassis_2.SetTargetCurrent(0.0f);
    g_motor_chassis_3.SetTargetCurrent(0.0f);
    g_motor_chassis_4.SetTargetCurrent(0.0f);

    static const osThreadAttr_t kChassisTaskAttr = {
        .name = "chassis_task",
        .stack_size = 1024,
        .priority = (osPriority_t) osPriorityNormal
    };
    osThreadNew(Chassis::TaskEntry, this, &kChassisTaskAttr);
}

void Chassis::TaskEntry(void *argument)
{
    Chassis *self = static_cast<Chassis *>(argument);
    self->Task();
}

void Chassis::Exit()
{
    // 模式切换/退出时清 PID，避免积分残留
    pid_omega_1_.reset();
    pid_omega_2_.reset();
    pid_omega_3_.reset();
    pid_omega_4_.reset();

    g_motor_chassis_1.SetTargetCurrent(0.0f);
    g_motor_chassis_2.SetTargetCurrent(0.0f);
    g_motor_chassis_3.SetTargetCurrent(0.0f);
    g_motor_chassis_4.SetTargetCurrent(0.0f);
}

/**
 * @brief 云台系速度 → 底盘系速度 旋转变换
 * @param yaw_angle 云台相对于底盘的偏航角（逆时针为正）
 */
void Chassis::RotationMatrixTransform()
{
    // 旋转矩阵变换
    target_vx_in_chassis_ = cosf(yaw_angle_) * target_vx_in_gimbal_ - sinf(yaw_angle_) * target_vy_in_gimbal_;
    target_vy_in_chassis_ = sinf(yaw_angle_) * target_vx_in_gimbal_ + cosf(yaw_angle_) * target_vy_in_gimbal_;
}

void Chassis::KinematicsInverseResolution()
{
    // 1) 运动学逆解：得到四轮目标角速度(rad/s)
    // 注：这里沿用你原来的 0.707107 系数；具体含义取决于轮子安装方向与坐标系定义。
    target_wheel_omega_1_rad_s_ = (+0.707107f * target_vx_in_chassis_ - 0.707107f * target_vy_in_chassis_) + target_velocity_rotation_;
    target_wheel_omega_2_rad_s_ = (-0.707107f * target_vx_in_chassis_ - 0.707107f * target_vy_in_chassis_) + target_velocity_rotation_;
    target_wheel_omega_3_rad_s_ = (-0.707107f * target_vx_in_chassis_ + 0.707107f * target_vy_in_chassis_) + target_velocity_rotation_;
    target_wheel_omega_4_rad_s_ = (+0.707107f * target_vx_in_chassis_ + 0.707107f * target_vy_in_chassis_) + target_velocity_rotation_;

    // 2) 速度环 PID：omega(rad/s) -> current(A)
    {
        const float now = g_motor_chassis_1.now_omega_rad_s();
        const float out_a = pid_omega_1_.update(target_wheel_omega_1_rad_s_, now);
        g_motor_chassis_1.SetTargetCurrent(out_a);
    }
    {
        const float now = g_motor_chassis_2.now_omega_rad_s();
        const float out_a = pid_omega_2_.update(target_wheel_omega_2_rad_s_, now);
        g_motor_chassis_2.SetTargetCurrent(out_a);
    }
    {
        const float now = g_motor_chassis_3.now_omega_rad_s();
        const float out_a = pid_omega_3_.update(target_wheel_omega_3_rad_s_, now);
        g_motor_chassis_3.SetTargetCurrent(out_a);
    }
    {
        const float now = g_motor_chassis_4.now_omega_rad_s();
        const float out_a = pid_omega_4_.update(target_wheel_omega_4_rad_s_, now);
        g_motor_chassis_4.SetTargetCurrent(out_a);
    }
}

void Chassis::OutputToMotor()
{
    g_motor_chassis_1.UpdateSlot();
    g_motor_chassis_2.UpdateSlot();
    g_motor_chassis_3.UpdateSlot();
    g_motor_chassis_4.UpdateSlot();
    actuator::drivers::DjiC6xxMin::FlushGroup(g_motor_chassis_1);
}
void Chassis::Task()
{
    // 订阅模式与底盘输入
    Sub<orb::RcControl> rc_sub(orb::rc_control);
    orb::RcControl rc{};

    bool last_enable = false;

    for (;;) {
        (void)rc_sub.copy(rc);

        // 新 RcControl 暂不包含开关：默认一直 enable（或由上层另加 topic 控制）
        const bool enable = true;
        if (enable != last_enable) {
            Exit();
            last_enable = enable;
        }

        // 将 rc 映射为底盘指令
        orb::ChassisCmd cmd{};
        cmd.enable = enable;
        cmd.vx = rc.chassis_x_speed * kChassisSpeed;
        cmd.vy = rc.chassis_y_speed * kChassisSpeed;
        cmd.wz = rc.chassis_rotation_speed * kChassisSpeed;

        
        // 向外发布（可选）
        orb::chassis_cmd.publish(cmd);

        if (!cmd.enable) {
            OutputToMotor();

            orb::ChassisState st{};
            st.enable = false;
            st.cmd_vx = cmd.vx;
            st.cmd_vy = cmd.vy;
            st.cmd_wz = cmd.wz;
            orb::chassis_state.publish(st);

            osDelay(1);
            continue;
        }

        // 更新底盘目标（底盘坐标系）
        target_vx_in_chassis_ = cmd.vx;
        target_vy_in_chassis_ = cmd.vy;
        target_velocity_rotation_ = cmd.wz;

        KinematicsInverseResolution();
        OutputToMotor();

        orb::ChassisState st{};
        st.enable = true;
        st.cmd_vx = cmd.vx;
        st.cmd_vy = cmd.vy;
        st.cmd_wz = cmd.wz;
        st.target_omega_1 = target_wheel_omega_1_rad_s_;
        st.target_omega_2 = target_wheel_omega_2_rad_s_;
        st.target_omega_3 = target_wheel_omega_3_rad_s_;
        st.target_omega_4 = target_wheel_omega_4_rad_s_;

        {
            ChassisWheelOmega msg{};
            msg.omega_1_rad_s = g_motor_chassis_1.now_omega_rad_s();
            msg.omega_2_rad_s = g_motor_chassis_2.now_omega_rad_s();
            msg.omega_3_rad_s = g_motor_chassis_3.now_omega_rad_s();
            msg.omega_4_rad_s = g_motor_chassis_4.now_omega_rad_s();
            chassis_wheel_omega.publish(msg);

            st.now_omega_1 = msg.omega_1_rad_s;
            st.now_omega_2 = msg.omega_2_rad_s;
            st.now_omega_3 = msg.omega_3_rad_s;
            st.now_omega_4 = msg.omega_4_rad_s;
        }

        orb::chassis_state.publish(st);

        osDelay(1);
    }
}

