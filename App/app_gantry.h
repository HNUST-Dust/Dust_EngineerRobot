#pragma once
#include "dvc_motor_cubemars.h"
#include "dvc_motor_dji.h"
class Gantry {
public:
    MotorCubemars motor_z_axis_left_;
    MotorCubemars motor_z_axis_right_;
    MotorDjiC620 motor_x_axis_left_;
    MotorDjiC620 motor_x_axis_right_;
    MotorDjiC610 motor_y_axis_;

    float virtual_z_distance_ = 0.0f;

    void Init();
    void Task();
    void Exit();

    void XAxisMoveInDistance(float distance);
    void YAxisMoveInDistance(float distance);
    void ZAxisMoveInDistance(float distance);

    void XAxisMoveInSpeed(float speed);
    void YAxisMoveInSpeed(float speed);
    void ZAxisMoveInSpeed(float speed);
    
    static constexpr float Z_AXIS_SENSITIVITY = 0.002f;

private:
    enum class ZAxisControlMode : uint8_t {
        Distance,
        Speed,
    };

    static constexpr float X_AXIS_DISTANCE_LIMIT = 10.0f; // distance
    static constexpr float Y_AXIS_DISTANCE_LIMIT = 10.0f; // distance
    static constexpr float Z_AXIS_DISTANCE_LIMIT = 20.0f;  // distance

    static constexpr float X_AXIS_SPEED_LIMIT = 20.0f; // speed
    static constexpr float Y_AXIS_SPEED_LIMIT = 20.0f; // speed
    static constexpr float Z_AXIS_SPEED_LIMIT = 50.0f; // speed

    static constexpr float Z_AXIS_TORQUE_LIMIT = 20.0f; // treated as "current" output at app layer
    static constexpr float GANTRY_TASK_DT = 0.01f;       // 10ms, 100Hz

    ZAxisControlMode z_axis_mode_ = ZAxisControlMode::Distance;
    float z_axis_speed_cmd_ = 0.0f;

    alg::Pid z_left_pid_angle_;
    alg::Pid z_left_pid_omega_;
    alg::Pid z_right_pid_angle_;
    alg::Pid z_right_pid_omega_;

    
    static void TaskEntry(void *param);

};