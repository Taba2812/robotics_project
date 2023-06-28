#include "ros/ros.h"
#include "libraries/robot.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "RobotTest_Node");
    ros::NodeHandle nh;

    ur5::Robot robot;

    std::cout << "Direct Kinematic" << std::endl;
    std::cout << robot.getEE().matrix << std::endl << std::endl;
    std::cout << robot.computeDirect().matrix << std::endl << std::endl;

    //std::cout << (robot.translateEndEffector(Eigen::Vector3d(0, 0, 0.1))).matrix << std::endl;

    std::cout << "Inverse Kinematic" << std::endl;
    ur5::JointAngles angles = robot.computeInverse(robot.getEE());
    std::cout << robot.computeDirect().matrix << std::endl << std::endl;

    std::cout << "------------------------------" << std::endl;
    std::cout << robot.translateEndEffector(Eigen::Vector3d(0,0,1)).matrix << std::endl << std::endl;

    return EXIT_SUCCESS;
}