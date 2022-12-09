#include "headers/position_control.h"

Eigen::Matrix<double, JOINTS, 1> joint_position_array;

void get_position(const std_msgs::Float64MultiArray::ConstPtr& msg){
    for(int i=0; i<JOINTS; i++){
        joint_position_array[i] = msg->data[i];
    }
}

int main(int argc, char **argv){
    ros::init(argc, argv, "position_control");
    ros::NodeHandle n;
    ros::Rate loop_rate(LOOP_RATE);

    ros::Subscriber joint_sub = n.subscribe("joint_publisher", QUEUE_SIZE, get_position);
    ros::Publisher joint_pub = n.advertise<std_msgs::Float64MultiArray>("joint_subscriber", LOOP_RATE);

    for(int i=0; i<2; i++){
        ros::spinOnce();
        loop_rate.sleep();
    }

    while(ros::ok()){
        
    }

    return 0;
}