#include "headers/position_control.h"

/*
for msg_idx in range(len(msg.name)):
    for joint_idx in range(len(self.joint_names)):
        if self.joint_names[joint_idx] == msg.name[msg_idx]:
            self.q[joint_idx] = msg.position[msg_idx]
*/

void get_position(const sensor_msgs::JointState::ConstPtr& js){
    for(int i=0; i<JOINTS; i++){
        q[i] = js->position[i];
    }
}

int main(int argc, char **argv){
    ros::init(argc, argv, "position_control");
    ros::NodeHandle nh;
    ros::Rate loop_rate(LOOP_RATE);

    ros::Subscriber joint_sub = nh.subscribe("/ur5/joint_states", QUEUE_SIZE, get_position);
    ros::Publisher joint_pub = nh.advertise<std_msgs::Float64MultiArray>("/ur5/joint_group_pos_controller/command", QUEUE_SIZE);

    for(int i=0; i<2; i++){
        ros::spinOnce();
        loop_rate.sleep();
    }

    while(ros::ok()){
        //compute
    }

    return 0;
}