#include "headers/position_control.h"

void get_joint(const sensor_msgs::JointState::ConstPtr& js){
    for(int i=0; i<JOINTS; i++){
        for(int j=0; j<JOINTS; j++){
            if(joint_names[j] == js->name.at(i)){
                q(i) = js->position[i];
            }
        }
    }
}

void get_position(const std_msgs::Float64MultiArray::ConstPtr& xyz){
    bp(0) = xyz->data[0];
    bp(1) = xyz->data[1];
    bp(2) = xyz->data[2];
}

int main(int argc, char **argv){

    ros::init(argc, argv, "position_control");
    ros::NodeHandle nh;
    ros::Rate loop_rate(LOOP_RATE);

    //publishers and subscribers
    ros::Subscriber joint_sub = nh.subscribe("/ur5/joint_states", QUEUE_SIZE, get_joint);
    ros::Publisher joint_pub = nh.advertise<std_msgs::Float64MultiArray>("/ur5/joint_group_pos_controller/command", QUEUE_SIZE);
    ros::Subscriber vision_sub = nh.subscribe("block_position", QUEUE_SIZE, get_position);
    ros::Publisher vision_pub = nh.advertise<std_msgs::Bool>("state", QUEUE_SIZE);

    //Environment components
    EndEffector ee;
    Destination block_start, block_end;
    JointConfiguration jc;
    std_msgs::Float64MultiArray msg;
    std_msgs::Bool status;
    std_msgs::Float64MultiArray home;

    home.data = {0, 0, 0};
    status.data = true;

    //Make sure we have received proper joint angles already
    for(int i=0; i<2; i++){
        ros::spinOnce();
        loop_rate.sleep();
    }

    //get default position
    ee.compute_direct(q);
    for(int i=0; i<3; i++){
        home.data[i] = ee.get_position()[i];
    }

    while(ros::ok()){
        vision_pub.publish(status);
        status.data = false;

        while(status.data == false){
            sleep(1);
        }

        Destination d(bp);
        block_start.compute_inverse(ee);

        //motion planning
        Path p;

        jc = block_start.get_joint_angles();

        for(int i=0; i<JOINTS; i++){
            msg.data[i] = jc(i);
        }

        //Have to figure out how to wait for the end of the movement
        
        //move to block
        joint_pub.publish(msg);

        //move the block in the desired position
        joint_pub.publish(block_end);

        //homing
        joint_pub.publish(home);
    }

    return 0;
}