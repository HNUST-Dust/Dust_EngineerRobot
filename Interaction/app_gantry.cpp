#include "app_gantry.h"
#include "dvc_motor_cubemars.h"
#include "dvc_motor_dji.h"
#include "cmsis_os2.h"
#include "cmsis_os.h"
#include "projdefs.h"

void Gantry::Init() {
    
    motor_z_axis_left_.Init(&hfdcan2,0x00,0x01,ANGLE_CONTROL);
    motor_z_axis_right_.Init(&hfdcan1,0x00,0x01,ANGLE_CONTROL);
    motor_z_axis_left_.SetKd(0.8);
    motor_z_axis_right_.SetKd(0.8);

    motor_x_axis_left_.pid_omega_.Init(1.0f,0.0f,0.0f);
    motor_x_axis_right_.pid_omega_.Init(1.0f,0.0f,0.0f);
    motor_y_axis_.pid_omega_.Init(
        1.0f,
        0.0f,
        0.0f,
        0.0f,
        9.0f,
        9.0f,
        0.001f
    );

    motor_x_axis_left_.pid_angle_.Init(5.0f,0.0f,0.1f);
    motor_x_axis_right_.pid_angle_.Init(5.0f,0.0f,0.1f);
    motor_y_axis_.pid_angle_.Init(
        30.0f,
        0.15f,
        0.0f,
        1.0f,
        29.0f,
        29.0f,
        0.001f
    );
    
    motor_x_axis_left_.Init(&hfdcan2, MOTOR_DJI_ID_0x201, MOTOR_DJI_CONTROL_METHOD_OMEGA);
    motor_x_axis_right_.Init(&hfdcan2, MOTOR_DJI_ID_0x202, MOTOR_DJI_CONTROL_METHOD_OMEGA);
    motor_y_axis_.Init(&hfdcan2, MOTOR_DJI_ID_0x203, MOTOR_DJI_CONTROL_METHOD_ANGLE);

    motor_z_axis_left_.CanSendSaveZero();
    motor_z_axis_right_.CanSendSaveZero();
    osDelay(pdMS_TO_TICKS(1000));

    motor_z_axis_left_.CanSendEnter();
    motor_z_axis_right_.CanSendEnter();

    static const osThreadAttr_t kGantryTaskAttr = {
        .name = "gantry_task",
        .stack_size = 512,
        .priority = (osPriority_t) osPriorityNormal
    };
    osThreadNew(Gantry::TaskEntry, this, &kGantryTaskAttr);
}

void Gantry::TaskEntry(void *argument) {
    Gantry *self = static_cast<Gantry *>(argument);
    self->Task();
}

void Gantry::Exit() {
    motor_z_axis_left_.CanSendExit();
    motor_z_axis_right_.CanSendExit();
    motor_x_axis_left_.SetTargetOmega(0);
    motor_x_axis_right_.SetTargetOmega(0);
    motor_y_axis_.SetTargetAngle(0);
}

void Gantry::XAxisMove(float distance) {
    motor_x_axis_left_.SetTargetAngle(distance);
    motor_x_axis_right_.SetTargetAngle(-distance);
}

void Gantry::YAxisMove(float distance) {
    motor_y_axis_.SetTargetAngle(distance);
}

void Gantry::ZAxisMove(float distance) {
    motor_z_axis_left_.SetAngle(distance);
    motor_z_axis_right_.SetAngle(-distance);
}

void Gantry::XAxisMoveInSpeed(float speed) {
    motor_x_axis_left_.SetTargetOmega(speed);
    motor_x_axis_right_.SetTargetOmega(-speed);
}

void Gantry::ZAxisMoveInSpeed(float speed) {
    motor_z_axis_left_.SetOmega(speed);
    motor_z_axis_right_.SetOmega(-speed);
}
void Gantry::Task() {
    for(;;) {
        motor_x_axis_left_.CalculatePeriodElapsedCallback();
        motor_x_axis_right_.CalculatePeriodElapsedCallback();
        motor_y_axis_.CalculatePeriodElapsedCallback();
        can_send_data(&hfdcan2, 0x200, g_can2_0x200_tx_data, 8);

        motor_z_axis_left_.CalculatePeriodElapsedCallback();
        motor_z_axis_left_.CanSendEnter();
        motor_z_axis_right_.CalculatePeriodElapsedCallback();
        motor_z_axis_right_.CanSendEnter();
        osDelay(pdMS_TO_TICKS(10)); // 100Hz
    }
}