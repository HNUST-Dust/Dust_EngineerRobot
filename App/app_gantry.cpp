#include "app_gantry.h"
#include "dvc_motor_cubemars.h"
#include "dvc_motor_dji.h"
#include "cmsis_os2.h"
#include "cmsis_os.h"
#include "projdefs.h"

void Gantry::Init() {

    motor_z_axis_left_.Init(&hfdcan2,0x00,0x01,ANGLE_CONTROL);
    motor_z_axis_right_.Init(&hfdcan1,0x00,0x01,ANGLE_CONTROL);

    {
        alg::PidConfig cfg;
        cfg.kp = 8.0f;
        cfg.ki = 0.0f;
        cfg.kd = 0.0f;
        cfg.kf = 0.0f;
        cfg.i_out_max = Z_AXIS_SPEED_LIMIT;
        cfg.out_max = Z_AXIS_SPEED_LIMIT;
        cfg.dt = GANTRY_TASK_DT;
        // cfg.d_lpf_tau = 0.005f;
        z_left_pid_angle_.configure(cfg);
        z_right_pid_angle_.configure(cfg);
    }
    {
        alg::PidConfig cfg;
        cfg.kp = 0.3f;
        cfg.ki = 0.0f;
        cfg.kd = 0.0f;
        cfg.kf = 0.0f;
        cfg.i_out_max = Z_AXIS_TORQUE_LIMIT;
        cfg.out_max = Z_AXIS_TORQUE_LIMIT;
        cfg.dt = GANTRY_TASK_DT;
        z_left_pid_omega_.configure(cfg);
        z_right_pid_omega_.configure(cfg);
    }

    z_axis_mode_ = ZAxisControlMode::Distance;
    virtual_z_distance_ = 0.0f;
    z_axis_speed_cmd_ = 0.0f;

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
    motor_y_axis_.Init(&hfdcan3, MOTOR_DJI_ID_0x201, MOTOR_DJI_CONTROL_METHOD_OMEGA);

    motor_z_axis_left_.CanSendSaveZero();
    motor_z_axis_right_.CanSendSaveZero();
    osDelay(pdMS_TO_TICKS(1000));

    motor_z_axis_left_.CanSendEnter();
    motor_z_axis_right_.CanSendEnter();

    // App 层闭环输出扭矩(视作电流指令)，关闭电机内部的 Kp/Kd 阻抗项
    motor_z_axis_left_.SetKp(0.0f);
    motor_z_axis_left_.SetKd(0.0f);
    motor_z_axis_left_.SetAngle(0.0f);
    motor_z_axis_left_.SetOmega(0.0f);
    motor_z_axis_left_.SetTorque(0.0f);

    motor_z_axis_right_.SetKp(0.0f);
    motor_z_axis_right_.SetKd(0.0f);
    motor_z_axis_right_.SetAngle(0.0f);
    motor_z_axis_right_.SetOmega(0.0f);
    motor_z_axis_right_.SetTorque(0.0f);

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
    motor_z_axis_left_.SetTorque(0.0f);
    motor_z_axis_right_.SetTorque(0.0f);
    motor_x_axis_left_.SetTargetOmega(0);
    motor_x_axis_right_.SetTargetOmega(0);
    motor_y_axis_.SetTargetOmega(0);
}

constexpr float Clamp(float value, float min, float max) {
    if (value < min) {
        return min;
    } else if (value > max) {
        return max;
    } else {
        return value;
    }
}

void Gantry::XAxisMoveInDistance(float distance) {
    distance = Clamp(distance, -X_AXIS_DISTANCE_LIMIT, X_AXIS_DISTANCE_LIMIT);
    motor_x_axis_left_.SetTargetAngle(distance);
    motor_x_axis_right_.SetTargetAngle(-distance);
}

void Gantry::YAxisMoveInDistance(float distance) {
    distance = Clamp(distance, -Y_AXIS_DISTANCE_LIMIT, Y_AXIS_DISTANCE_LIMIT);
    motor_y_axis_.SetTargetAngle(distance);
}

void Gantry::ZAxisMoveInDistance(float distance) {
    distance = Clamp(distance, -Z_AXIS_DISTANCE_LIMIT, Z_AXIS_DISTANCE_LIMIT);

    z_axis_mode_ = ZAxisControlMode::Distance;
    z_axis_speed_cmd_ = 0.0f;
    virtual_z_distance_ = distance;
    z_left_pid_angle_.reset();
    z_left_pid_omega_.reset();
    z_right_pid_angle_.reset();
    z_right_pid_omega_.reset();
}

void Gantry::XAxisMoveInSpeed(float speed) {
    speed = Clamp(speed, -X_AXIS_SPEED_LIMIT, X_AXIS_SPEED_LIMIT);
    motor_x_axis_left_.SetTargetOmega(speed);
    motor_x_axis_right_.SetTargetOmega(-speed);
}

void Gantry::YAxisMoveInSpeed(float speed) {
    speed = Clamp(speed, -Y_AXIS_SPEED_LIMIT, Y_AXIS_SPEED_LIMIT);
    motor_y_axis_.SetTargetOmega(speed);
}

void Gantry::ZAxisMoveInSpeed(float speed) {
    speed = Clamp(speed, -Z_AXIS_SPEED_LIMIT, Z_AXIS_SPEED_LIMIT);

    z_axis_mode_ = ZAxisControlMode::Speed;
    z_axis_speed_cmd_ = speed;
}

void Gantry::Task() {
    for(;;) {
        motor_x_axis_left_.CalculatePeriodElapsedCallback();
        motor_x_axis_right_.CalculatePeriodElapsedCallback();
        can_send_data(&hfdcan2, 0x200, g_can2_0x200_tx_data, 8);

        motor_y_axis_.CalculatePeriodElapsedCallback();
        //can_send_data(&hfdcan3, 0x200, g_can3_0x200_tx_data, 8);
        
        if (z_axis_mode_ == ZAxisControlMode::Speed) {
            virtual_z_distance_ += z_axis_speed_cmd_ * GANTRY_TASK_DT;
            virtual_z_distance_ = Clamp(virtual_z_distance_, -Z_AXIS_DISTANCE_LIMIT, Z_AXIS_DISTANCE_LIMIT);
        }

        const float z_target_left = virtual_z_distance_;
        const float z_target_right = -virtual_z_distance_;

        const float z_omega_target_left = z_left_pid_angle_.update(z_target_left, motor_z_axis_left_.GetAngle());
        const float z_omega_target_right = z_right_pid_angle_.update(z_target_right, motor_z_axis_right_.GetAngle());

        const float z_torque_left = z_left_pid_omega_.update(z_omega_target_left, motor_z_axis_left_.GetOmega());
        const float z_torque_right = z_right_pid_omega_.update(z_omega_target_right, motor_z_axis_right_.GetOmega());

        motor_z_axis_left_.SetKp(0.0f);
        motor_z_axis_left_.SetKd(0.0f);
        motor_z_axis_left_.SetAngle(0.0f);
        motor_z_axis_left_.SetOmega(0.0f);
        motor_z_axis_left_.SetTorque(z_torque_left);
        motor_z_axis_left_.CalculatePeriodElapsedCallback();
        motor_z_axis_left_.CanSendEnter();

        motor_z_axis_right_.SetKp(0.0f);
        motor_z_axis_right_.SetKd(0.0f);
        motor_z_axis_right_.SetAngle(0.0f);
        motor_z_axis_right_.SetOmega(0.0f);
        motor_z_axis_right_.SetTorque(z_torque_right);
        motor_z_axis_right_.CalculatePeriodElapsedCallback();
        motor_z_axis_right_.CanSendEnter();

        osDelay(pdMS_TO_TICKS(10)); // 100Hz
    }
}