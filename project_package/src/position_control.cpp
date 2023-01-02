#include "headers/position_control.h"

void get_joint(const sensor_msgs::JointState::ConstPtr& js){
    for(int i=0; i<JOINTS; i++){
        q(i) = js->position[i];
    }
}

void get_link(const gazebo_msgs::LinkStates::ConstPtr& ls){
    int counter = 0;
    for(auto a : ls->pose){
        if(counter < LINKS){
            p[counter][0] = a.position.x;
            p[counter][1] = a.position.y;
            p[counter][2] = a.position.z;
            counter++;
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
    ros::Subscriber link_sub = nh.subscribe("/gazebo/link_states", QUEUE_SIZE, get_link);
    ros::Publisher joint_pub = nh.advertise<std_msgs::Float64MultiArray>("/ur5/joint_group_pos_controller/command", QUEUE_SIZE);
    ros::Subscriber vision_sub = nh.subscribe("block_position", QUEUE_SIZE, get_position);
    ros::Publisher vision_pub = nh.advertise<std_msgs::Bool>("state", QUEUE_SIZE);

    //Environment components
    EndEffector ee;
    Destination d;
    JointConfiguration jc;

    //Make sure we have received proper joint angles already
    for(int i=0; i<2; i++){
        ros::spinOnce();
        loop_rate.sleep();
    }

    while(ros::ok()){
        ee.compute_direct(q);
        vision_pub.publish(true);
        Destination d(bp);
        d.compute_inverse(ee);

        //motion planning

        jc = d.get_joint_angles();
        joint_pub.publish(jc);
    }

    return 0;
}