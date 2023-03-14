#include "libraries/bezier_curve.h"
#include "ros/ros.h"

int main (int argc, char **argv) {

    ros::init(argc, argv, "MotionPlanning_Test");

    Bezier::Node destination = {6,6,6};
    int segments = 20;

    Bezier::Curve test_curve(destination, segments);

    std::cout << "Starting Bezier Points:" << std::endl;
    for (int i = 0; i <= segments; i++) {
        std::cout << "[" << i << "] ";
        Bezier::printNode(test_curve.getNext());
    }
}