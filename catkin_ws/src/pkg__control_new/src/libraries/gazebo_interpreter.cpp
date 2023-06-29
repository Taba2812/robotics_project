#include "gazebo_interpreter.h"

Gazebo::Interpreter::Interpreter() {}

ur5::JointAngles Gazebo::Interpreter::parseJointState(const sensor_msgs::JointState::ConstPtr &ja) {
    ur5::JointAngles _ja;

    for(int i=0; i < NO_JOINTS; i++){
        for(int j=0; j< NO_JOINTS; j++){
            if(ur5::jointNames[j] == ja->name.at(i)){
                _ja(i) = ja->position[i];
            }
        }
    }

    correct(_ja); 

    return _ja;
}

ur5::JointAngles Gazebo::Interpreter::parseArray(const std_msgs::Float64MultiArray::ConstPtr &ja) {
    ur5::JointAngles _ja;

    for (int i = 0; i < NO_JOINTS; i++) {
        _ja(i) = ja->data.at(i);
    }

    return _ja;
}

std_msgs::Float64MultiArray Gazebo::Interpreter::createJointMessage(const ur5::JointAngles &ja) {
    std_msgs::Float64MultiArray msg;
    msg.data.resize(NO_JOINTS);

    ur5::JointAngles _ja = ja;

    for(int i=0; i< NO_JOINTS; i++) {
        msg.data.at(i) = ja(i);
    }

    return msg;
}

std_msgs::Float32MultiArray Gazebo::Interpreter::createJointMessage32(const ur5::JointAngles &ja) {
    std_msgs::Float32MultiArray msg;
    msg.data.resize(NO_JOINTS);

    ur5::JointAngles _ja = ja;

    for(int i=0; i< NO_JOINTS; i++) {
        msg.data.at(i) = ja(i);
    }

    return msg;
}

void Gazebo::Interpreter::correct(ur5::JointAngles &ja) {
    std::swap(ja(0), ja(2));
}

bool Gazebo::Interpreter::hasReachedDestination(ur5::JointAngles joints, float dt) {
    bool reached = true;
    for (int i = 0; i < NO_JOINTS; i++) {
        //std::cout << "Joint#" << i << "=" << joints[i] << " | Dest#" << i << "=" << this->destination[i] << std::endl;
        //std::cout << "Check#1=" << (joints[i] < (this->destination[i] + dt)) << " | Check#2=" << (joints[i] > (this->destination[i] - dt)) << std::endl;
        if (!(joints[i] < (this->destination[i] + dt) && joints[i] > (this->destination[i] - dt)))
            reached = false;
    }
    //std::cout << "------------------------------------" << std::endl;
    return reached;
}