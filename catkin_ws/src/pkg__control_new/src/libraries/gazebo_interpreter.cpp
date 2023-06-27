#include "gazebo_interpreter.h"

Gazebo::Interpreter::Interpreter() {}

ur5::JointAngles Gazebo::Interpreter::parseJointState(const sensor_msgs::JointState::ConstPtr &ja) {
    ur5::JointAngles _ja;

    for(int i=0; i<ur5::noJoints; i++){
        for(int j=0; j<ur5::noJoints; j++){
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

    for (int i = 0; i < ur5::noJoints; i++) {
        _ja(i) = ja->data.at(i);
    }

    return _ja;
}

ur5::JointAngles Gazebo::Interpreter::sanitizeJoints(ur5::JointAngles joints) {

    if (joints[0] < ur5::joint0_min ) {joints[0] = ur5::joint0_min; std::cout << "Sanitized Destination" << std::endl;};
    if (joints[0] > ur5::joint0_max ) {joints[0] = ur5::joint0_max; std::cout << "Sanitized Destination" << std::endl;};

    if (joints[1] < ur5::joint1_min ) {joints[1] = ur5::joint1_min; std::cout << "Sanitized Destination" << std::endl;};
    if (joints[1] > ur5::joint1_max ) {joints[1] = ur5::joint1_max; std::cout << "Sanitized Destination" << std::endl;};

    if (joints[2] < ur5::joint2_min ) {joints[2] = ur5::joint2_min; std::cout << "Sanitized Destination" << std::endl;};
    if (joints[2] > ur5::joint2_max ) {joints[2] = ur5::joint2_max; std::cout << "Sanitized Destination" << std::endl;};

    if (joints[3] < ur5::joint3_min ) {joints[3] = ur5::joint3_min; std::cout << "Sanitized Destination" << std::endl;};
    if (joints[3] > ur5::joint3_max ) {joints[3] = ur5::joint3_max; std::cout << "Sanitized Destination" << std::endl;};

    if (joints[4] < ur5::joint4_min ) {joints[4] = ur5::joint4_min; std::cout << "Sanitized Destination" << std::endl;};
    if (joints[4] > ur5::joint4_max ) {joints[4] = ur5::joint4_max; std::cout << "Sanitized Destination" << std::endl;};

    if (joints[5] < ur5::joint5_min ) {joints[5] = ur5::joint5_min; std::cout << "Sanitized Destination" << std::endl;};
    if (joints[5] > ur5::joint5_max ) {joints[5] = ur5::joint5_max; std::cout << "Sanitized Destination" << std::endl;};

    return joints;
}

std_msgs::Float64MultiArray Gazebo::Interpreter::createJointMessage(const ur5::JointAngles &ja) {
    std_msgs::Float64MultiArray msg;
    msg.data.resize(ur5::noJoints);

    ur5::JointAngles _ja = ja;

    for(int i=0; i<ur5::noJoints; i++) {
        msg.data.at(i) = ja(i);
    }

    return msg;
}

void Gazebo::Interpreter::correct(ur5::JointAngles &ja) {
    std::swap(ja(0), ja(2));
}

bool Gazebo::Interpreter::hasReachedDestination(ur5::JointAngles joints, float dt) {
    bool reached = true;
    for (int i = 0; i < ur5::noJoints; i++) {
        if (!(joints[i] < (this->destination[i] + dt) && joints[i] > (this->destination[i] - dt)))
            reached = false;
    }

    return reached;
}