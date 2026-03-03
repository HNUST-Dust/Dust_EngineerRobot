#pragma once

#include "../Algorithm/control/alg_pid.h"

class Gantry {
public:
    static Gantry& Instance()
    {
        static Gantry instance;
        return instance;
    }


    // ---- 外置 PID ----
    // X left/right: omega -> current（速度控制，无位置环）
    alg::Pid x_left_pid_omega_;
    alg::Pid x_right_pid_omega_;

    // Y: omega -> current（速度控制，无位置环）
    alg::Pid y_pid_omega_;

    // Z left/right: angle -> omega_ref（位置环；速度环交给电机 MIT 的 kd）
    alg::Pid z_left_pid_angle_;
    alg::Pid z_right_pid_angle_;

    // ---- 目标量（单位：rad / rad/s）----
    float z_target_angle_rad_ = 0.0f;

    float x_target_omega_rad_s_ = 0.0f;
    float y_target_omega_rad_s_ = 0.0f;

    float virtual_z_distance_ = 0.0f;

    void Init();
    void Task();
    void Exit();

    void XAxisMoveInDistance(float distance);  // 保留：外部可直接设置目标位置（暂不使用）
    void YAxisMoveInDistance(float distance);
    void ZAxisMoveInDistance(float distance);

    void XAxisMoveInSpeed(float speed);
    void YAxisMoveInSpeed(float speed);
    void ZAxisMoveInSpeed(float speed);

    static constexpr float Z_AXIS_SENSITIVITY = 0.0002f;



private:
    Gantry() = default;
    Gantry(const Gantry&) = delete;
    Gantry& operator=(const Gantry&) = delete;

    static constexpr float X_AXIS_DISTANCE_LIMIT = 5.0f;
    static constexpr float Y_AXIS_DISTANCE_LIMIT = 10.0f;
    static constexpr float Z_AXIS_DISTANCE_LIMIT = 5.0f;

    static constexpr float X_AXIS_SPEED_LIMIT = 20.0f;
    static constexpr float Y_AXIS_SPEED_LIMIT = 20.0f;
    static constexpr float Z_AXIS_SPEED_LIMIT = 20.0f;

    static void TaskEntry(void *param);
};

// 兼容层（可逐步移除）：保留旧函数名，内部转发到 Instance()
inline Gantry& GantryInstance() { return Gantry::Instance(); }