#ifndef APP_ROBOT_H_
#define APP_ROBOT_H_
// app
#include "app_chassis.h"
#include "user_lib.h"
#include "low_pass_filter.hpp"
// module
#include "dvc_MCU_comm.h"
#include "supercap.h"
#include "debug_tools.h"


#define CHASSIS_SPEED                    15.0f
#define CHASSIS_SPIN_SPEED               30.0f


class Robot
{
public:
    // 调试工具
    DebugTools debug_tools_;
    // 底盘
    Chassis chassis_;

    // 底盘陀螺仪
    Imu imu_;

    void Init();
    void Task();
protected:
    
    // 机器人等级
    int32_t robot_level_ = 1;
    static void TaskEntry(void *param);
};

#endif // !APP_ROBOT_H_