#ifndef GAZEBO_INT
#define GAZEBO_INT

#include "robot.h"

namespace Gazebo {

    typedef std_msgs::Float64MultiArray JointMessage;

    class Interpreter {
        private:
           ur5::JointAngles jointAngles;
           JointMessage jointMessage;
        public:
            Interpreter();

            ur5::JointAngles parseJointState(const sensor_msgs::JointState::ConstPtr &ja);
            std_msgs::Float64MultiArray createJointMessage(const ur5::JointAngles &ja);

            void correct(ur5::JointAngles &ja);

            ur5::JointAngles getJointAngles();
            JointMessage getJointMessage();
    };

}

#endif