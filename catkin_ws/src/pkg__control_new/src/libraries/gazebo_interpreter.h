#ifndef GAZEBO_INT
#define GAZEBO_INT

#include "robot.h"

namespace Gazebo {

    typedef std_msgs::Float64MultiArray JointMessage;

    class Interpreter {
        public:
            Interpreter();

            ur5::JointAngles parseJointState(const sensor_msgs::JointState::ConstPtr &ja);
            std_msgs::Float64MultiArray createJointMessage(const ur5::JointAngles &ja);

            void correct(ur5::JointAngles &ja);
    };

}

#endif