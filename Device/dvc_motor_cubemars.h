#pragma once

#include "bsp_can.h"
#include <cstdint>
#include "alg_pid.h"

enum MotorCubemarsControlMethod {
   TORQUE_CONTROL,
   OMEGA_CONTROL,
   ANGLE_CONTROL, 
};

struct MotorCubemarsCanRxData {
    uint8_t can_id;
    uint16_t angle_reverse;
    uint8_t omega_11_4;
    uint8_t omega_3_0_torque_11_8;
    uint8_t torque_7_0;

} __attribute__((packed));

struct MotorCubemarsRxData {
    float now_angle;
    float now_omega;
    float now_torque;
    uint32_t pre_encoder;
    int32_t total_encoder;
    int32_t total_round;
    float now_angle_noncumulative;
};

struct MotorCubemarsCanTxData {
    uint16_t control_angle_reverse;
    uint8_t control_omega_11_4;
    uint8_t control_omega_3_0_k_p_11_8;
    uint8_t k_p_7_0;
    uint8_t k_d_11_4;
    uint8_t k_d_3_0_control_torque_11_8;
    uint8_t control_torque_7_0;
} __attribute__((packed));

class MotorCubemars {
public:
    Pid pid_angle_;
    
    void Init(
        FDCAN_HandleTypeDef *hcan,
        uint8_t can_rx_id,
        uint8_t can_tx_id,
        MotorCubemarsControlMethod control_method =
        ANGLE_CONTROL,
        float angle_max = 12.5f,
        float omega_max = 50.0f,
        float torque_max = 65.0f
    );

    inline void SetTorque(float torque) {
        control_torque_ = torque;
    };

    inline void SetOmega(float omega) {
        control_omega_ = omega;
    };

    inline void SetAngle(float angle) {
        control_angle_ = angle;
    };

    inline void SetKp(float k_p) {
        k_p_ = k_p;
    };
    inline void SetKd(float k_d) {
        k_d_ = k_d;
    };
    
    inline float GetTorque() {
        return rx_data_.now_torque;
    };

    inline float GetOmega() {
        return rx_data_.now_omega;
    };

    inline float GetAngle() {
        return rx_data_.now_angle;
    };

    void CanRxCpltCallback(uint8_t *rx_data_);
    void CanSendEnter();
    void CanSendExit();
    void CanSendSaveZero();
    void SendPeriodElapsedCallback();
    void PidCalculate();
    void CalculatePeriodElapsedCallback();
    void Output();
    void pack_cmd(float p_des, float v_des, float kp, float kd, float t_ff);
    void unpack_reply(uint8_t *rx_data_);
protected:
    // 绑定的CAN
    CanManageObject *can_manage_object_;
    // 收数据绑定的CAN ID, 与上位机驱动参数Master_ID保持一致
    uint16_t can_rx_id_;
    // 发数据绑定的CAN ID, 是上位机驱动参数can_id加上控制模式的偏移量
    uint16_t can_tx_id_;
    // 发送缓冲区
    uint8_t tx_data_[8];
    // 电机对外接口数据
    MotorCubemarsRxData rx_data_;

    // 最大位置, 与上位机控制幅值PMAX保持一致
    float angle_max_;
    // 最大速度, 与上位机控制幅值VMAX保持一致
    float omega_max_;
    // 最大扭矩, 与上位机控制幅值TMAX保持一致
    float torque_max_;
    
    MotorCubemarsControlMethod control_method_ =
        ANGLE_CONTROL;

    // 角度, rad
    float control_angle_ = 0.0f;
    // 角速度, rad/s
    float control_omega_ = 0.0f;
    // 扭矩, Nm
    float control_torque_ = 0.0f;
    // k_p_, 0~500
    float k_p_ = 0.0f;
    // k_d_, 0~5
    float k_d_ = 0.0f;

    void DataProcess();
};
