#include "headers/direct_kinematics.h"
#include "headers/inverse_kinematics.h"
//#include "state_machine/process.h"
//#include "state_machine/concrete_states.h"

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
    //initialize node
    ros::init(argc, argv, "ur5Main");
    ros::NodeHandle nh;
    ros::Rate loopRate(LOOP_RATE);

    //setup ros params
    std::string joint_docker_in, joint_docker_out, detection_req, detection_res;
    nh.getParam("Docker_Joint_In", joint_docker_in);
    nh.getParam("Docker_Joint_Out", joint_docker_out);
    nh.getParam("Core2Det_Req", detection_req);
    nh.getParam("Det2Core_Res", detection_res);

    //publishers and subscribers
    ros::Publisher jointPub = nh.advertise<std_msgs::Float64MultiArray>(joint_docker_out, QUEUE_SIZE);
    ros::Publisher visionPub = nh.advertise<std_msgs::Bool>(detection_req, QUEUE_SIZE);
    ros::Subscriber jointSub = nh.subscribe(joint_docker_in, QUEUE_SIZE, getJoint);
    ros::Subscriber visionSub = nh.subscribe(detection_res, QUEUE_SIZE, getPosition);

    //environment components
    EndEffector ee;
    Destination d, home;
    FinalDestination fd;
    std_msgs::Bool msgVision;
    std_msgs::Bool msgMotion;

    //default values
    processStatus.data = false;
    bool gripper = false;


    //make sure we have received proper joint angles already
    for(int i=0; i<2; i++){
        ros::spinOnce();
        loopRate.sleep();
    }

    //get default position
    ee.computeDirect(q);
    for(int i=0; i<3; i++){
        home.setPosition(ee.getPosition());
    }

    int currentState = 0;

    while(ros::ok()){
        switch(currentState){
            case WAITING:
                std::cout << "\n[WAITING] press any key to advance\n";
                std::cin.get();
                currentState = VISION;
                msgVision.data = true;
                visionPub.publish(msgVision);
            break;

            case VISION:
                std::cout << "\n[VISION] object recognition\n";
                while(!processStatus.data) usleep(WAITING_TIME);
                currentState = POSITION;
            break;

            case POSITION:
                std::cout << "\n[POSITION]";
                if(!gripper){
                    std::cout << " home to block\n";
                    d.setPosition(bp);
                } else {
                    std::cout << " block to destination\n";
                    d.setPosition(fd);
                }
                d.computeInverse(ee);
                currentState = MOTION;
                msgMotion.data = true;
                //publish on motion topic
            break;

            case MOTION:
                std::cout << "\n[MOTION] path planning\n";
                //update with proper topic
                while(1){
                    usleep(WAITING_TIME);
                }
                if(!gripper) currentState = TO_BLOCK;
                else currentState = TO_FINAL;
            break;

            case TO_BLOCK:
                std::cout << "\n[TO_BLOCK] moving to detected block\n";
                for(int i=0; i<POINTS; i++){
                    jointPub.publish(d.getMessage());
                }
                //attach dynamic link for block
                gripper = true;
                currentState = POSITION;
            break;

            case TO_FINAL:
                std::cout << "\n[TO_FINAL] moving to final destination\n";
                for(int i=0; i<POINTS; i++){
                    jointPub.publish(d.getMessage());
                }
                //detach dynamic link for block
                gripper = false;
                currentState = HOMING;
            break;

            case HOMING:
                std::cout << "\n[HOMING] going home\n";
                msgMotion.data = true;
                //publish topic
                while(1){
                    usleep(WAITING_TIME);
                }
                for(int i=0; i<POINTS; i++){
                    jointPub.publish(home.getMessage());
                }
                currentState = VISION;
            break;

            default: std::cout << "\n[ERROR] invalid operation\n" << std::endl;
        }
    }

    return 0;
}