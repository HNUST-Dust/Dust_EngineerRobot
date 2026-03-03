#include "app_gantry.h"

#include "cmsis_os2.h"

#include "../communication_topic/topic_pubsub.hpp"
#include "../communication_topic/rc_control_topics.hpp"
#include "../communication_topic/gantry_state_topics.hpp"
#include "../Device/motors/motor_instances.hpp"

#include <cmath>

namespace {
using actuator::instances::g_motor_x_axis_left;
using actuator::instances::g_motor_x_axis_right;
using actuator::instances::g_motor_y_axis;
using actuator::instances::g_motor_z_axis_left;
using actuator::instances::g_motor_z_axis_right;
}

static constexpr float Clamp(float value, float min, float max) {
    if (value < min) return min;
    if (value > max) return max;
    return value;
}

void Gantry::Init() {
    // 电机对象由 system_startup.cpp 的 startup_thread 统一实例化并注入到 Gantry 单例里。

    // ---- PID config ----
    // X omega（速度控制：摇杆直接给 omega 目标，速度环输出电流）
    {
        alg::PidConfig cfg;
        cfg.kp = 2.0f;
        cfg.ki = 0.0f;
        cfg.kd = 0.0f;
        cfg.kf = 0.0f;
        cfg.i_out_max = 9.0f;
        cfg.out_max = 9.0f;
        cfg.dt = 0.01f;
        x_left_pid_omega_.configure(cfg);
        x_right_pid_omega_.configure(cfg);
    }

    // Y omega（速度控制）
    {
        alg::PidConfig cfg;
        cfg.kp = 2.0f;
        cfg.ki = 0.0f;
        cfg.kd = 0.0f;
        cfg.kf = 0.0f;
        cfg.i_out_max = 9.0f;
        cfg.out_max = 9.0f;
        cfg.dt = 0.01f;
        y_pid_omega_.configure(cfg);
    }

    // Z angle -> omega_ref
    {
        alg::PidConfig cfg;
        cfg.kp = 5.0f;
        cfg.ki = 0.0f;
        cfg.kd = 0.1f;
        cfg.kf = 0.0f;
        cfg.i_out_max = 29.0f;
        cfg.out_max = 50.0f; // omega_ref(rad/s)
        cfg.dt = 0.01f;
        z_left_pid_angle_.configure(cfg);
        z_right_pid_angle_.configure(cfg);
    }

    // bring up Z motors (save zero + enter)
    g_motor_z_axis_left.BringUpDefault();
    g_motor_z_axis_left.SetTarget(0.0f, 0.0f, 0.0f);
    g_motor_z_axis_left.PublishMitTx(0.0f, 0.8f);

    g_motor_z_axis_right.BringUpDefault();
    g_motor_z_axis_right.SetTarget(0.0f, 0.0f, 0.0f);
    g_motor_z_axis_right.PublishMitTx(0.0f, 0.8f);

    static const osThreadAttr_t kGantryTaskAttr = {
        .name = "gantry_task",
        .stack_size = 1024,
        .priority = (osPriority_t) osPriorityNormal
    };
    osThreadNew(Gantry::TaskEntry, this, &kGantryTaskAttr);
}

void Gantry::TaskEntry(void *argument) {
    Gantry *self = static_cast<Gantry *>(argument);
    self->Task();
}

void Gantry::Exit() {
    g_motor_z_axis_left.Exit();
    g_motor_z_axis_right.Exit();

    x_target_omega_rad_s_ = 0.0f;
    y_target_omega_rad_s_ = 0.0f;
}

void Gantry::XAxisMoveInDistance(float distance) {
    (void)distance; // 速度控制模式下不使用位置目标
}

void Gantry::YAxisMoveInDistance(float distance) {
    (void)distance;
}

void Gantry::ZAxisMoveInDistance(float distance) {
    distance = Clamp(distance, -Z_AXIS_DISTANCE_LIMIT, Z_AXIS_DISTANCE_LIMIT);
    z_target_angle_rad_ = distance;
}

void Gantry::XAxisMoveInSpeed(float speed) {
    speed = Clamp(speed, -X_AXIS_SPEED_LIMIT, X_AXIS_SPEED_LIMIT);
    x_target_omega_rad_s_ = speed;
}

void Gantry::YAxisMoveInSpeed(float speed) {
    speed = Clamp(speed, -Y_AXIS_SPEED_LIMIT, Y_AXIS_SPEED_LIMIT);
    y_target_omega_rad_s_ = speed;
}

void Gantry::ZAxisMoveInSpeed(float speed) {
    (void)speed; // 积累逻辑已移至 DR16 侧，此接口不再使用
}

void Gantry::Task()
{
    Sub<orb::RcControl> rc_sub(orb::rc_control);
    orb::RcControl rc{};

    for (;;) {
        (void)rc_sub.copy(rc);


        const float x_in = rc.gantry_x_axis_distance;
        const float y_in = rc.gantry_y_axis_distance;
        const float z_in = rc.gantry_z_axis_distance;

        XAxisMoveInSpeed(x_in * 3.0f);
        YAxisMoveInSpeed(y_in * 3.0f);
        ZAxisMoveInDistance(z_in * 0.1f);  // z_in 已是 DR16 侧积累好的目标角度

        // ---- X axis PID（速度控制：omega → current）----
        {
            const float now_omega = g_motor_x_axis_left.now_omega_rad_s();
            const float current = x_left_pid_omega_.update(x_target_omega_rad_s_, now_omega);
            g_motor_x_axis_left.SetTargetCurrent(current);
            g_motor_x_axis_left.UpdateSlot();
        }
        {
            const float now_omega = g_motor_x_axis_right.now_omega_rad_s();
            const float current = x_right_pid_omega_.update(-x_target_omega_rad_s_, now_omega);
            g_motor_x_axis_right.SetTargetCurrent(current);
            g_motor_x_axis_right.UpdateSlot();
        }
        actuator::drivers::DjiC6xxMin::FlushGroup(g_motor_x_axis_left);

        // ---- Y axis PID（速度控制：omega → current）----
        {
            const float now_omega = g_motor_y_axis.now_omega_rad_s();
            const float current = y_pid_omega_.update(y_target_omega_rad_s_, now_omega);
            g_motor_y_axis.SetTargetCurrent(current);
            g_motor_y_axis.UpdateSlot();
        }
        // CAN3/0x200 组帧：y_axis(slot1) + wrist_left(slot2) + wrist_right(slot3)
        // Arm 任务已通过 UpdateSlot() 更新 wrist 槽位，此处统一 Flush
        actuator::drivers::DjiC6xxMin::FlushGroup(g_motor_y_axis);

        // ---- Z axis (Cubemars) ----
        {
            const float now_angle = g_motor_z_axis_left.now_angle_rad();
            const float omega_ref = z_left_pid_angle_.update(z_target_angle_rad_, now_angle);
            g_motor_z_axis_left.SetTarget(z_target_angle_rad_, omega_ref, 0.0f);
            g_motor_z_axis_left.PublishMitTx(0.0f, 0.8f);
        }
        {
            const float now_angle = g_motor_z_axis_right.now_angle_rad();
            const float omega_ref = z_right_pid_angle_.update(-z_target_angle_rad_, now_angle);
            g_motor_z_axis_right.SetTarget(-z_target_angle_rad_, omega_ref, 0.0f);
            g_motor_z_axis_right.PublishMitTx(0.0f, 0.8f);
        }

        // publish state
        orb::GantryState st{};
        st.enable           = true;
        st.in_x_speed       = x_in;
        st.in_y_speed       = y_in;
        st.in_z_speed       = z_in;

        st.x_target_omega   = x_target_omega_rad_s_;
        st.y_target_omega   = y_target_omega_rad_s_;
        st.z_target_angle   = z_target_angle_rad_;
        st.virtual_z_distance = z_target_angle_rad_; // 积累在 DR16 侧，此处等同

        st.x_left_now_omega  = g_motor_x_axis_left.now_omega_rad_s();
        st.x_right_now_omega = g_motor_x_axis_right.now_omega_rad_s();
        st.y_now_omega       = g_motor_y_axis.now_omega_rad_s();

        st.z_left_now_angle  = g_motor_z_axis_left.now_angle_rad();
        st.z_right_now_angle = g_motor_z_axis_right.now_angle_rad();
        st.z_left_omega      = g_motor_z_axis_left.now_omega_rad_s();
        st.z_left_total_angle = g_motor_z_axis_left.now_total_angle_rad();
        st.z_left_torque     = g_motor_z_axis_left.now_torque_nm();
        orb::gantry_state.publish(st);

        osDelay(10); // 100Hz
    }
}

// （已改为 Gantry::Instance();GantryInstance() 在头文件中 inline 转发）