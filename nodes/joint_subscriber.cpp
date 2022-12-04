#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/JointState.h"
#include <Eigen/Dense>

typedef  Eigen::Matrix<double, 6, 1> JointStateVector;

void print_values(const sensor_msgs::JointState &sensor_msg){
	std::cout << "I received: " << sensor_msg << "\n";
}

int main(int argc, char **argv){
	ros::init(argc, argv, "joint_subscriber");
	ros::NodeHandle n;
	ros::Subscriber sub = n.subscribe("joint_publisher", 1000, print_values);
	ros::spin();

	return 0;
}