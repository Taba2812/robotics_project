#include "ros/ros.h"
#include "libraries/robot.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "RobotTest_Node");
    ros::NodeHandle nh;

    ur5::Robot robot;

    std::cout << robot.getEE_position() << "\n\n" << robot.getEE_orientation_matrix() << std::endl << std::endl;

    //std::cout << (robot.translateEndEffector(Eigen::Vector3d(0, 0, 0.1))).matrix << std::endl;

    robot.computeInverse(robot.getEE());

    return EXIT_SUCCESS;
}