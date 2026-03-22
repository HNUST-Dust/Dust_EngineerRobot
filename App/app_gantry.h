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
    
    static constexpr float Z_AXIS_SENSITIVITY = 0.01f;
    static constexpr float Z_AXIS_DISTANCE_LIMIT = 5.0f;  // distance

private:
    alg::Pid z_left_pid_angle_;
    alg::Pid z_right_pid_angle_;
    // Z 轴控制量统一使用“距离”（单位与 virtual_z_distance_ / Z_AXIS_DISTANCE_LIMIT 保持一致）
    float z_target_distance_ = 0.0f;

    // 距离->电机角度(rad) 换算系数：rad = distance * Z_AXIS_RAD_PER_DISTANCE
    // TODO: 按丝杆导程/减速比等实际机构标定该值
    static constexpr float Z_AXIS_RAD_PER_DISTANCE = 1.0f;
    static constexpr float X_AXIS_DISTANCE_LIMIT = 10.0f; // distance
    static constexpr float Y_AXIS_DISTANCE_LIMIT = 10.0f; // distance

    static constexpr float X_AXIS_SPEED_LIMIT = 20.0f; // speed
    static constexpr float Y_AXIS_SPEED_LIMIT = 20.0f; // speed
    static constexpr float Z_AXIS_SPEED_LIMIT = 20.0f; // speed
    
    static void TaskEntry(void *param);
};