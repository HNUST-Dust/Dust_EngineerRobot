#include "dvc_motor_cubemars.h"
#include "bsp_can.h"
#include "alg_math.h"
#include <cstdint>

uint8_t kMotorCANMessageEnter[8] = {
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC
};
uint8_t kMotorCanMessageExit[8] = {
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD
};
uint8_t kMotorCANMessageSaveZero[8] = {
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0XFE
};
void MotorCubemars::Init(
    FDCAN_HandleTypeDef *hcan,
    uint8_t can_rx_id,
    uint8_t can_tx_id,
    MotorCubemarsControlMethod control_method,
    float angle_max,
    float omega_max,
    float torque_max
) {
    if (hcan->Instance == FDCAN1)
    {
        can_manage_object_ = &g_can1_manage_object;
    }
    else if (hcan->Instance == FDCAN2)
    {
        can_manage_object_ = &g_can2_manage_object;
    }
    else if (hcan->Instance == FDCAN3)
    {
        can_manage_object_ = &g_can3_manage_object;
    }
    can_tx_id_ = can_tx_id;
    can_rx_id_ = can_rx_id;
    control_method_ = control_method;
    angle_max_ = angle_max;
    omega_max_ = omega_max;
    torque_max_ = torque_max;
}

/**
 * @brief CAN通信接收回调函数
 *
 * @param rx_data_ 接收的数据
 */
void MotorCubemars::CanRxCpltCallback(uint8_t *rx_data_)
{
    DataProcess();
}

/**
 * @brief 发送使能电机
 *
 */
void MotorCubemars::CanSendEnter()
{
    can_send_data(can_manage_object_->can_handler, can_tx_id_, kMotorCANMessageEnter, 8);
}

/**
 * @brief 发送失能电机
 *
 */
void MotorCubemars::CanSendExit()
{
    can_send_data(can_manage_object_->can_handler, can_tx_id_, kMotorCanMessageExit, 8);
}

/**
 * @brief 发送保存当前位置为零点
 *
 */
void MotorCubemars::CanSendSaveZero()
{
    can_send_data(can_manage_object_->can_handler, can_tx_id_, kMotorCANMessageSaveZero, 8);
}



void MotorCubemars::Output() {
    switch (control_method_) {
        case (TORQUE_CONTROL) :
        {
            MotorCubemarsCanTxData *tmp_buffer = (MotorCubemarsCanTxData *)tx_data_;
            
            uint16_t tmp_angle, tmp_omega, tmp_torque, tmp_k_p, tmp_k_d;

            tmp_angle = math_float_to_int(control_angle_, 0, angle_max_, 0x7fff, (1 << 16) - 1);
            tmp_omega = math_float_to_int(control_omega_, 0, omega_max_, 0x7ff, (1 << 12) - 1);
            tmp_torque = math_float_to_int(control_torque_, 0, torque_max_, 0x7ff, (1 << 12) - 1);
            tmp_k_p = math_float_to_int(k_p_, 0, 500.0f, 0, (1 << 12) - 1);
            tmp_k_d = math_float_to_int(k_d_, 0, 5.0f, 0, (1 << 12) - 1);

            tmp_buffer->control_angle_reverse = math_endian_reverse_16(&tmp_angle, nullptr);
            tmp_buffer->control_omega_11_4 = tmp_omega >> 4;
            tmp_buffer->control_omega_3_0_k_p_11_8 = ((tmp_omega & 0x0f) << 4) | (tmp_k_p >> 8);
            tmp_buffer->k_p_7_0 = tmp_k_p & 0xff;
            tmp_buffer->k_d_11_4 = tmp_k_d >> 4;
            tmp_buffer->k_d_3_0_control_torque_11_8 = ((tmp_k_d & 0x0f) << 4) | (tmp_torque >> 8);
            tmp_buffer->control_torque_7_0 = tmp_torque & 0xff;
            can_send_data(can_manage_object_->can_handler, can_tx_id_, tx_data_, 8);
            break;
        }
        default:
            break;
    }
}

void MotorCubemars::DataProcess()
{
    // 数据处理过程
    int32_t delta_encoder;
    uint16_t tmp_encoder, tmp_omega, tmp_torque;
    MotorCubemarsCanRxData *tmp_buffer = (MotorCubemarsCanRxData *)can_manage_object_->rx_buffer.data;

    // 电机ID不匹配, 则不进行处理
    if(tmp_buffer->can_id != (can_tx_id_ & 0x0f))
    {
        return;
    }

    // 处理大小端
    math_endian_reverse_16((void *)&tmp_buffer->angle_reverse, &tmp_encoder);
    tmp_omega = (tmp_buffer->omega_11_4 << 4) | (tmp_buffer->omega_3_0_torque_11_8 >> 4);
    tmp_torque = ((tmp_buffer->omega_3_0_torque_11_8 & 0x0f) << 8) | (tmp_buffer->torque_7_0);

    // 计算圈数与总角度值
    delta_encoder = tmp_encoder - rx_data_.pre_encoder;
    if (delta_encoder < -(1 << 15))
    {
        // 正方向转过了一圈
        rx_data_.total_round++;
    }
    else if (delta_encoder > (1 << 15))
    {
        // 反方向转过了一圈
        rx_data_.total_round--;
    }
    rx_data_.total_encoder = rx_data_.total_round * (1 << 16) + tmp_encoder - ((1 << 15) - 1);

    // 计算电机本身信息
    rx_data_.now_angle_noncumulative = ((float)((tmp_encoder / 65535.0f) * (angle_max_ * 2.0f)) - angle_max_);
    rx_data_.now_angle = (float)(rx_data_.total_encoder) / (float)((1 << 16) - 1) * angle_max_ * 2.0f;
    rx_data_.now_omega = math_int_to_float(tmp_omega, 0x7ff, (1 << 12) - 1, 0, omega_max_);
    rx_data_.now_torque = math_int_to_float(tmp_torque, 0x7ff, (1 << 12) - 1, 0, torque_max_);

    // 存储预备信息
    rx_data_.pre_encoder = tmp_encoder;
}
//发包时所有的数都要经以下函数转化成整型数之后再发给电机。 
int float_to_uint(float x, float x_min, float x_max, unsigned int bits){
    /// Converts a float to an unsigned int, given range and number of bits 
    float span = x_max - x_min;
    if(x < x_min) x = x_min; 
    else if(x > x_max) x = x_max; 
    return (int) ((x- x_min)*((float)((1<<bits)/span)));
}

void MotorCubemars::pack_cmd(float p_des, float v_des, float kp, float kd, float t_ff){
    
    float P_MIN =-12.5f; 
    float P_MAX =12.5f; 
    float V_MIN =-50.0f;//30 
    float V_MAX =50.0f; 
    float T_MIN =-65.0f;//18 
    float T_MAX =65.0f; 
    float Kp_MIN =0;
    float Kp_MAX =500.0f; 
    float Kd_MIN =0;
    float Kd_MAX =5.0f; 
    float Test_Pos=0.0f; 
    int p_int ;
    int v_int; 
    int kp_int ; 
    int kd_int; 
    int t_int ;

    p_des = fminf(fmaxf(P_MIN, p_des), P_MAX); 
    v_des = fminf(fmaxf(V_MIN, v_des), V_MAX); 
    kp = fminf(fmaxf(Kp_MIN, kp), Kp_MAX); 
    kd = fminf(fmaxf(Kd_MIN, kd), Kd_MAX); 
    t_ff = fminf(fmaxf(T_MIN, t_ff), T_MAX);
    p_int = float_to_uint(p_des, P_MIN, P_MAX, 16); 
    v_int= float_to_uint(v_des, V_MIN, V_MAX, 12); 
    kp_int = float_to_uint(kp, Kp_MIN, Kp_MAX, 12); 
    kd_int = float_to_uint(kd, Kd_MIN, Kd_MAX, 12); 
    t_int= float_to_uint(t_ff, T_MIN, T_MAX, 12);

    tx_data_[0] = p_int>>8; //位置高 8 
    tx_data_[1] = p_int&0xFF; //位置低 8 
    tx_data_[2] = v_int>>4; //速度高 8 位
    tx_data_[3] = ((v_int&0xF)<<4)|(kp_int>>8); //速度低 4 位 KP 高 4 位 
    tx_data_[4] = kp_int&0xFF; //KP 低 8 位
    tx_data_[5] = kd_int>>4; //Kd 高 8 位 
    tx_data_[6] = ((kd_int&0xF)<<4)|(t_int>>8); //KP 低 4 位扭矩高 4 位 
    tx_data_[7] = t_int&0xff; //扭矩低 8 位

    can_send_data(can_manage_object_->can_handler, can_tx_id_, tx_data_, 8);

}


void MotorCubemars::SendPeriodElapsedCallback()
{
    // 电机在线, 正常控制
    math_constrain(&control_angle_, -angle_max_, angle_max_);
    math_constrain(&control_omega_, -omega_max_, omega_max_);
    math_constrain(&control_torque_, -torque_max_, torque_max_);
    math_constrain(&k_p_, 0.0f, 500.0f);
    math_constrain(&k_d_, 0.0f, 5.0f);

    //pack_cmd(control_angle_,control_omega_,k_p_,k_d_,control_torque_);
    Output();
}
void PidCalculate() {

}

void CalculatePeriodElapsedCallback() {

}

//收包时所有的数都要经以下函数转化成浮点型。 
float uint_to_float(int x_int, float x_min, float x_max, int bits){
    /// converts unsigned int to float, given range and number of bits /// 
    float span = x_max - x_min;
    float offset = x_min; 
    return ((float)x_int)*span/((float)((1<<bits)-1)) + offset;
}

void MotorCubemars::unpack_reply(uint8_t *rx_data){ 
    /// unpack ints from can buffer ///
    uint8_t *tmp_buffer = (uint8_t *)can_manage_object_->rx_buffer.data;

    float P_MIN =-12.5f; 
    float P_MAX =12.5f; 
    float V_MIN =-50.0f; 
    float V_MAX =50.0f; 
    float T_MIN =-65.0f; 
    float T_MAX =65.0f; 
    float Kp_MIN =0;
    float Kp_MAX =500.0f; 
    float Kd_MIN =0;
    float Kd_MAX =5.0f; 
    float Test_Pos=0.0f;

    int id = tmp_buffer[0]; //驱动 ID 号 
    int p_int = ( tmp_buffer[1]<<8)| tmp_buffer[2]; //电机位置数据 
    int v_int = ( tmp_buffer[3]<<4)|( tmp_buffer[4]>>4); //电机速度数据 
    int i_int = ((tmp_buffer[4]&0xF)<<8)| tmp_buffer[5]; //电机扭矩数据 


    /// convert ints to floats 
    // float p = uint_to_float(p_int, P_MIN, P_MAX, 16); 
    // float v = uint_to_float(v_int, V_MIN, V_MAX, 12); 
    // float i = uint_to_float(i_int, -T_MAX, T_MAX, 12); 
    // float T =T_int;

    if(id == 1){ 
        rx_data_.now_angle = uint_to_float(p_int, P_MIN, P_MAX, 16); 
        rx_data_.now_omega = uint_to_float(v_int, V_MIN, V_MAX, 12); 
        rx_data_.now_torque = uint_to_float(i_int, -T_MAX, T_MAX, 12); 
    }
}
