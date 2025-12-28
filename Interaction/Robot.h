#ifndef APP_ROBOT_H_
#define APP_ROBOT_H_
// app
#include "app_chassis.h"
#include "app_gantry.h"
#include "app_arm.h"
// module
#include "debug_tools.h"
#include "dr16.h"


#define CHASSIS_SPEED                    15.0f
#define CHASSIS_SPIN_SPEED               30.0f


class Robot
{
public:
    // 调试工具
    DebugTools debug_tools_;
    // 遥控器DR16
    DR16 dr16_;
    // 底盘
    Chassis chassis_;
    // 龙门架
    Gantry gantry_;
    // 机械臂
    Arm arm_;

    void Init();
    void Task();
protected:
    
    // 机器人等级
    int32_t robot_level_ = 1;
    static void TaskEntry(void *param);
};

#endif // !APP_ROBOT_H_