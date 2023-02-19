#include "headers/direct_kinematics.h"
#include "headers/inverse_kinematics.h"
#include "state_machine/process.h"
#include "state_machine/concrete_states.h"


void getJoint(const sensor_msgs::JointState::ConstPtr& js){
    for(int i=0; i<JOINTS; i++){
        for(int j=0; j<JOINTS; j++){
            if(jointNames[j] == js->name.at(i)){
                q(i) = js->position[i];
            }
        }
    }
}

void getPosition(const std_msgs::Float64MultiArray::ConstPtr& xyz){
    bp(0) = xyz->data[0];
    bp(1) = xyz->data[1];
    bp(2) = xyz->data[2];
    processStatus.data = true;
}

int main(int argc, char **argv){
    //state machine
    Process process;

    //initialize node
    ros::init(argc, argv, "ur5Main");
    ros::NodeHandle nh;
    ros::Rate loop_rate(LOOP_RATE);

    //publishers and subscribers
    ros::Publisher joint_pub = nh.advertise<std_msgs::Float64MultiArray>("/ur5/joint_group_pos_controller/command", QUEUE_SIZE);
    ros::Publisher vision_pub = nh.advertise<std_msgs::Bool>("Main_Bool", QUEUE_SIZE);
    ros::Subscriber joint_sub = nh.subscribe("/ur5/joint_states", QUEUE_SIZE, getJoint);
    ros::Subscriber vision_sub = nh.subscribe("Main_MultiArray", QUEUE_SIZE, getPosition);

    //environment components
    EndEffector ee;
    Destination d;
    FinalDestination fd;
    std_msgs::Float64MultiArray msg, home;

    //default values
    home.data = {0, 0, 0};

    //make sure we have received proper joint angles already
    for(int i=0; i<2; i++){
        ros::spinOnce();
        loop_rate.sleep();
    }

    //get default position
    ee.computeDirect(q);
    for(int i=0; i<3; i++){
        home.data[i] = ee.getPosition()[i];
    }

    process.processOn();

    while(ros::ok()){
        switch(process.getCurrentState() -> getCode()){
            //Waiting
            case 0 :
                vision_pub.publish(home);
                process.execute();
            break;

            //Vision
            case 1 :
                if(!process.getProcessStatus()) vision_pub.publish(true);
                process.execute();
            break;

            //Position
            case 2 :
                if(!gripper){
                    d.setPosition(bp);
                } else {
                    d.setPosition(fd);
                }
                d.computeInverse(ee);
                process.processOn();
                process.execute();
            break;
            
            //Motion
            case 3 :
                joint_pub.publish(d.getMessage());
                gripper = !gripper;
                process.processOn();
                process.execute();
            break;

            default: std::cout << "\nThere's been an error\n"; exit(0);
        }
    }

    return 0;
}

//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