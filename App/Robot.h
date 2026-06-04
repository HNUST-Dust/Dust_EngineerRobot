#ifndef APP_ROBOT_H_
#define APP_ROBOT_H_
// app
#include "app_chassis.h"
#include "app_gantry.h"
#include "app_arm.h"
#include "app_gimbal.h"
// module
#include "debug_tools.h"
#include "dr16.h"
#include "vt03.h"
#include "dvc_pwm_servo.h"

#include "stm32h7xx_hal_tim.h"
#include "tim.h"

static constexpr float kChassisSpeed = 20.0f;
static constexpr float kChassisSpinSpeed = 30.0f;


class Robot
{
public:
    // 调试工具
    DebugTools debug_tools_;
    // 遥控器DR16
    DR16 dr16_;
    VT03 vt03_;
    // 云台
    // Gimbal gimbal_;
    // 底盘
    Chassis chassis_;
    // 龙门架
    Gantry gantry_;
    // 机械臂
    Arm arm_;

    DvcPwmServo pitch_servo_{&htim1, TIM_CHANNEL_1, 0.5f, 2.5f, 180.0f, 20.0f};
    DvcPwmServo yaw_servo_{&htim1, TIM_CHANNEL_3, 0.5f, 2.5f, 270.0f, 20.0f};

    void Init();
    void Task();
protected:
    
    // 机器人等级
    int32_t robot_level_ = 1;

    
    static void TaskEntry(void *param);
};

#endif // !APP_ROBOT_H_