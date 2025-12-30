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

    void Init();
    void Task();
    void Exit();

    void XAxisMoveInDistance(float distance);
    void YAxisMoveInDistance(float distance);
    void ZAxisMoveInDistance(float distance);

    void XAxisMoveInSpeed(float speed);
    void YAxisMoveInSpeed(float speed);
    void ZAxisMoveInSpeed(float speed);

private:
    static constexpr float X_AXIS_DISTANCE_LIMIT = 10.0f; // distance
    static constexpr float Y_AXIS_DISTANCE_LIMIT = 10.0f; // distance
    static constexpr float Z_AXIS_DISTANCE_LIMIT = 5.0f;  // distance

    static constexpr float X_AXIS_SPEED_LIMIT = 20.0f; // speed
    static constexpr float Y_AXIS_SPEED_LIMIT = 20.0f; // speed
    static constexpr float Z_AXIS_SPEED_LIMIT = 20.0f; // speed

    static void TaskEntry(void *param);

};