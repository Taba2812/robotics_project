#include "headers/position_control.h"

void get_position(const sensor_msgs::JointState::ConstPtr& js){
    for(int i=0; i<JOINTS; i++){
        q[i] = js->position[i];
    }
}

void new_pose(double x, double y, double z, double roll, double pitch, double yaw){
    //code
}

int main(int argc, char **argv){
    ros::init(argc, argv, "position_control");
    ros::NodeHandle nh;
    ros::Rate loop_rate(LOOP_RATE);

    q.resize(6,1);

    ros::Subscriber joint_sub = nh.subscribe("/ur5/joint_states", QUEUE_SIZE, get_position);
    ros::Publisher joint_pub = nh.advertise<std_msgs::Float64MultiArray>("/ur5/joint_group_pos_controller/command", QUEUE_SIZE);

    for(int i=0; i<2; i++){
        ros::spinOnce();
        loop_rate.sleep();
    }

    while(ros::ok()){
        //various processes of nodes and topics
    }

    return 0;
}