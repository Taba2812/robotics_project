#include "headers/joint_subscriber.h"

char* joint_names[JOINTS] = {"shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"};
double q[JOINTS] = {0,0,0,0,0,0};

void receive_jstate(sensor_msgs::JointState::ConstPtr& msg){
    for(auto& m : msg->name){
        for(int i = 0; i<JOINTS; i++){
            if(joint_names[i] == m){
                q[i] = msg->position[i];
            }
        }
    }
}

int main(int argc, char **argv){
    ros::init(argc, argv, "joint_subscriber");
    ros::NodeHandle node;

    ros::Subscriber sub_jstate = node.subscribe("/ur5/joint_states", QUEUE_SIZE, receive_jstate);
    sleep(2);

    int loop_frequency = LOOP_FREQ;
    ros::Rate loop_rate = ros::Rate(loop_frequency);

    return 0;
}