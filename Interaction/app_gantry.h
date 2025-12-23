#pragma once
#include "dvc_motor_cubemars.h"
#include "dvc_motor_dji.h"
class Gantry {
public:
    MotorCubemars motor_z_axis_left_;
    MotorCubemars motor_z_axis_right_;
    MotorDjiC620 motor_x_axis_left_;
    MotorDjiC620 motor_x_axis_right_;


    void Init();
    void Task();
    void Exit();

private:
    static void TaskEntry(void *param);

};