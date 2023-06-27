#ifndef GAZEBO_INT
#define GAZEBO_INT

#include "robot.h"

namespace Gazebo {

    typedef std_msgs::Float64MultiArray JointMessage;

    class Interpreter {
        private:
            ur5::JointAngles destination;
            
        public:
            Interpreter();

            bool moving = false;
            void setDestination(ur5::JointAngles joints) {this->destination = joints;}
            bool hasReachedDestination(ur5::JointAngles joints, float dt);
            ur5::JointAngles parseJointState(const sensor_msgs::JointState::ConstPtr &ja);
            std_msgs::Float64MultiArray createJointMessage(const ur5::JointAngles &ja);
            std_msgs::Float64MultiArray publishDestination() {return this->createJointMessage(this->destination);};

            void correct(ur5::JointAngles &ja);
    };

}

#endif