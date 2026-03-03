#include "dji_c6xx.hpp"
#include "dm_mit.hpp"
#include "cubemars_mit.hpp"

namespace actuator::instances {

    inline actuator::drivers::DmMitMin g_claws;
    inline actuator::drivers::DmMitMin g_elbow_joint_pitch;
    inline actuator::drivers::DmMitMin g_elbow_joint_yaw;

}

namespace actuator::instances {
    inline actuator::drivers::DjiC6xxMin g_motor_chassis_1;
    inline actuator::drivers::DjiC6xxMin g_motor_chassis_2;
    inline actuator::drivers::DjiC6xxMin g_motor_chassis_3;
    inline actuator::drivers::DjiC6xxMin g_motor_chassis_4;

    inline actuator::drivers::DjiC6xxMin g_motor_x_axis_left;
    inline actuator::drivers::DjiC6xxMin g_motor_x_axis_right;
    inline actuator::drivers::DjiC6xxMin g_motor_y_axis;

    inline actuator::drivers::DjiC6xxMin g_wrist_joint_left;
    inline actuator::drivers::DjiC6xxMin g_wrist_joint_right;
}

namespace actuator::instances {
    inline actuator::drivers::CubemarsMitMin g_motor_z_axis_left;
    inline actuator::drivers::CubemarsMitMin g_motor_z_axis_right;
}