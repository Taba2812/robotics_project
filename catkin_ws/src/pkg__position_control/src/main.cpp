#include "headers/direct_kinematics.h"
#include "headers/inverse_kinematics.h"

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

    //environment components
    EndEffector ee;
    Destination d, home;
    FinalDestination fd;
    std_msgs::Bool msgVision;
    std_msgs::Bool msgMotion;

    //default values
    processStatus.data = false;
    bool gripper = false;

    //setup ros params
    int noSteps;
    std::string joint_docker_in, joint_docker_out, detection_req, detection_res, motion_req, motion_data, motion_res;
    
    nh.getParam("Docker_Joint_In", joint_docker_in);
    nh.getParam("Docker_Joint_Out", joint_docker_out);
    nh.getParam("Core2Det_Req", detection_req);
    nh.getParam("Det2Core_Res", detection_res);
    nh.getParam("Core2MP_Req", motion_req);
    nh.getParam("Core2MP_Data", motion_data);
    nh.getParam("MP2Core_Data", motion_res);
    nh.getParam("Steps", noSteps);

    //publishers and subscribers
    ros::Publisher jointPub = nh.advertise<std_msgs::Float64MultiArray>(joint_docker_out, QUEUE_SIZE);
    ros::Publisher visionPub = nh.advertise<std_msgs::Bool>(detection_req, QUEUE_SIZE);
    ros::Publisher motionPub = nh.advertise<std_msgs::Bool>(motion_req, QUEUE_SIZE);
    ros::Publisher dataPub = nh.advertise<std_msgs::Float64MultiArray>(motion_data, QUEUE_SIZE);

    ros::Subscriber jointSub = nh.subscribe<sensor_msgs::JointState>(joint_docker_in, QUEUE_SIZE, getJoint);
    ros::Subscriber visionSub = nh.subscribe<std_msgs::Float64MultiArray>(detection_res, QUEUE_SIZE, getPosition);

    int motionCounter = 0;
    auto getMotion = [&] (const std_msgs::Float32MultiArrayConstPtr &next_position) {
        motionCounter++;

        if (next_position->data == d.getDestination().data) {
            std::cout << "[Core] Destination reached sending RESET signal" << std::endl;
            std_msgs::Float32MultiArray reset;
            reset.data = {0,0,0,0};
            dataPub.publish(reset);
        } else {
            std_msgs::Bool next;
            next.data = true;
            motionPub.publish(next);
        }
    };

    ros::Subscriber motionSub = nh.subscribe<std_msgs::Float32MultiArray>(motion_res, QUEUE_SIZE, getMotion);

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
                motionPub.publish(msgMotion);
            break;

            case MOTION:
                std::cout << "\n[MOTION] path planning\n";
                while(1){
                    usleep(WAITING_TIME);
                }
                if(motionCounter <= noSteps) currentState = MOTION;
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