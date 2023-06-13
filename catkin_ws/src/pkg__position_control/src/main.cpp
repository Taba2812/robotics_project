#include "headers/inverse_kinematics.h"

#include <cmath>
#include <eigen3/Eigen/Geometry>
#include <iostream>
#include <vector>

#include "geometry_msgs/Pose.h"
#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Float64MultiArray.h"

#define _USE_MATH_DEFINES

//"configurationProvider": "ms-vscode.cmake-tools"

#define WAITING  0
#define JS       1
#define VISION   2
#define POSITION 3
#define MOTION   4
#define TO_BLOCK 5
#define TO_FINAL 6
#define HOMING   7

#define LOOP_RATE 100

typedef Eigen::Vector3d FinalDestination;

const ros::V_string jointNames = {"shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"};

JointAngles q;

int main(int argc, char **argv){
    sleep(1);

    //initialize node
    ros::init(argc, argv, "ur5Main");
    ros::NodeHandle nh;
    ros::Rate loopRate(LOOP_RATE);

    //setup ros params
    int noSteps, queue_size;
    std::string joint_docker_in, joint_docker_out, detection_req, detection_res, motion_req, motion_data, motion_res, js_data, js_new, js_req;
    
    nh.getParam("Docker_Joint_In", joint_docker_in);
    nh.getParam("Docker_Joint_Out", joint_docker_out);
    nh.getParam("Core2Det_Req", detection_req);
    nh.getParam("Det2Core_Res", detection_res);
    nh.getParam("Core2MP_Req", motion_req);
    nh.getParam("Core2MP_Data", motion_data);
    nh.getParam("MP2Core_Data", motion_res);
    nh.getParam("Core2JS_Req", js_req);
    nh.getParam("Core2JS_Data", js_new);
    nh.getParam("JS2Core_Data", js_data);
    nh.getParam("STEPS", noSteps);
    nh.getParam("Q_Size", queue_size);

    //environment components
    EndEffector ee;
    Destination d, home;
    FinalDestination fd;
    Eigen::Vector3d initialStep;
    Eigen::Vector3d finalStep;
    Eigen::Vector3d currentPosition;
    Eigen::Vector3d baseLink;
    Position blockPosition;

    Eigen::Vector3d ev3d;
    Eigen::Matrix3d em3d;

    //ros messages
    std_msgs::Bool msgVision;
    std_msgs::Bool msgJS;
    std_msgs::Float32MultiArray msgMotion;

    std_msgs::Float64MultiArray gazeboHome;
    gazeboHome.data.resize(JOINTS);

    gazeboHome.data.at(0) = -0.32;
    gazeboHome.data.at(1) = -0.78;
    gazeboHome.data.at(2) = -2.6;
    gazeboHome.data.at(3) = -1.63;
    gazeboHome.data.at(4) = -1.57;
    gazeboHome.data.at(5) = 3.49;

    //default values
    bool jointStatus = false;
    bool positionStatus = false;
    bool motionStatus = false;
    bool gripper = false;
    bool defaultPosition = true;
    bool goingHome = false;

    blockPosition = Position::Zero();
    msgMotion.data = {0,0,0,0,0,0,0};
    msgJS.data = true;
    baseLink << 0.5, 0.35, 1.75;

    //user interaction
    char c;

    // curve generation steps
    Eigen::Vector3d curveSteps[noSteps];
    Eigen::Vector3d curvePositions[noSteps];
    std_msgs::Float64MultiArray curveJoints[noSteps];
    

    //publishers
    ros::Publisher jointPub = nh.advertise<std_msgs::Float64MultiArray>(joint_docker_out, queue_size);
    // ros::Publisher jointPub = nh.advertise<sensor_msgs::JointState>(js_new, queue_size);
    ros::Publisher visionPub = nh.advertise<std_msgs::Bool>(detection_req, queue_size);
    ros::Publisher motionPub = nh.advertise<std_msgs::Bool>(motion_req, queue_size);
    ros::Publisher dataPub = nh.advertise<std_msgs::Float32MultiArray>(motion_data, queue_size);
    ros::Publisher requestPub = nh.advertise<std_msgs::Bool>(js_req, queue_size);


    #pragma region callbacks
    //lambda callbacks
    int motionCounter = 0;
    auto getMotion = [&] (const std_msgs::Float32MultiArrayConstPtr &next_position) {

        std::cout << "[" << motionCounter << "] ";    
        for(int i=0; i<3; i++) std::cout << next_position->data.at(i) << " ";
        std::cout << std::endl;

        if (next_position->data.at(0) == 0 &&
            next_position->data.at(1) == 0 &&
            next_position->data.at(2) == 0) {

            std::cout << "\n[Core] Destination reached sending RESET signal\n";
            std_msgs::Float32MultiArray reset;
            reset.data = {0,0,0,0};
            dataPub.publish(reset);

        } else {

            for(int i=0; i<3; i++){
                curveSteps[motionCounter](i) = next_position->data.at(i);
            }

            if(motionCounter < noSteps){
                motionCounter++;
                std_msgs::Bool next;
                next.data = true;
                motionPub.publish(next);
            } else motionCounter = 0;
        }

        if(motionCounter == 0) motionStatus = true;
    };

    auto getJoint = [&] (const sensor_msgs::JointState::ConstPtr &js) {
        if (jointStatus) {return;}
        std::cout << "\n[Main] received joint states\n";
        for(int i=0; i<JOINTS; i++){
            for(int j=0; j<JOINTS; j++){
                if(jointNames[j] == js->name.at(i)){
                    // std::cout << "[" << i << "] jsName = " << js->name.at(i) << " jointName = " << jointNames[j];
                    q(i) = js->position[i];
                    std::cout << q(i) << " "; 
                }
            }
            std::cout << "\n";
        }
        jointStatus = true;
    };

    auto getPosition = [&] (const std_msgs::Float32MultiArray::ConstPtr &xyz) {
        std::cout << "\n[Core] received Detection results data\n";

        blockPosition(0) = baseLink(0) - xyz->data[0];
        blockPosition(1) = baseLink(1) - xyz->data[1];
        blockPosition(2) = baseLink(2) - xyz->data[2];

        for(int i=0; i<3; i++){
            std::cout << "bp(" << i << ") = " << blockPosition(i) << "\n";
        }

        positionStatus = true;
    };

    //subscribers
    ros::Subscriber motionSub = nh.subscribe<std_msgs::Float32MultiArray>(motion_res, queue_size, getMotion);
    ros::Subscriber jointSub = nh.subscribe<sensor_msgs::JointState>(joint_docker_in, queue_size, getJoint);
    // ros::Subscriber jointSub = nh.subscribe<sensor_msgs::JointState>(js_data, queue_size, getJoint);
    ros::Subscriber visionSub = nh.subscribe<std_msgs::Float32MultiArray>(detection_res, queue_size, getPosition);

    #pragma endregion callbacks
    
    int currentState = WAITING;

    #pragma region stateMachine
    while(ros::ok()){
        switch(currentState){

            case WAITING:
                // jointPub.publish(gazeboHome);
                std::cout << "\n";
                std::cout << "[Waiting] press any key to advance: ";
                std::cin >> c;

                currentState = JS;

                // currentState = WAITING;
            break;

            case JS:
                std::cout << "\n[JointStates] retrieving joint states\n";

                // jointStatus = false;
                // requestPub.publish(msgJS);

                jointStatus = false;

                while(!jointStatus) ros::spinOnce();
                ee.computeDirect(q);

                if(defaultPosition){
                    home.setPosition(ee.getPosition());
                    home.setOrientation(ee.getOrientation());
                    defaultPosition = false;
                }

                home.setPosition({-0.207579, 0.063110, -0.630587});
                ee.setPosition({-0.207579, 0.063110, -0.630587});

                std::cout << "\nHome: \n" << home.getPosition() << "\n";
                std::cout << "\n" << home.getOrientation() << "\n\n";
                
                currentState = VISION;
                msgVision.data = true;
                visionPub.publish(msgVision);

                // currentState = WAITING;

            break;

            case VISION:
                std::cout << "\n[Vision] Waiting for detection\n";

                while(!positionStatus) ros::spinOnce();
                currentState = POSITION;

                // currentState = WAITING;
            break;

            case POSITION:
                std::cout << "\n[Position] Evaluating the destination...";

                if(gripper) {
                    std::cout << "going from block to destination\n";
                    d.setPosition(fd);
                    d.setOrientation(em3d);
                } else if(goingHome) {
                    std::cout << "homing\n";
                    d.setPosition(home.getPosition());
                    d.setOrientation(em3d);
                } else {
                    std::cout << "going from home to block\n";
                    d.setPosition(blockPosition);
                    d.setOrientation(em3d);
                }

                d.computeInverse(ee);

                std::cout << "\nDestination: \n" << d.getJointAngles() << "\n";

                // if(d.isWithinJointLimits(d.getJointAngles())) std::cout << "\n[Position] Destination reachable!\n";
                // else std::cout << "\n[Error] Destination not reachable!\n";

                currentState = MOTION;
            break;

            case MOTION:
                std::cout << "\n[Motion] computing path planning...\n";
                initialStep << ee.getPosition();
                d.setPosition(blockPosition);
                finalStep << d.getPosition();

                // std::cout << "Initial step: " << initialStep << "\n";
                // std::cout << "Final step: " << finalStep << "\n\n"; 

                msgMotion.data.at(0) = initialStep(0);
                msgMotion.data.at(1) = initialStep(1);
                msgMotion.data.at(2) = initialStep(2);
                msgMotion.data.at(3) = noSteps;
                msgMotion.data.at(4) = finalStep(0);
                msgMotion.data.at(5) = finalStep(1);
                msgMotion.data.at(6) = finalStep(2);

                
                dataPub.publish(msgMotion);

                while(!motionStatus) ros::spinOnce();

                currentPosition << initialStep;

                for(int i=0; i<noSteps; i++) {
                    curvePositions[i] << currentPosition + curveSteps[i];
                    currentPosition << curvePositions[i];

                    std::cout << curvePositions[i] << "\n\n";

                    ee.setPosition(curvePositions[i]);
                    d.computeInverse(ee);
                    
                    curveJoints[i].data.resize(JOINTS);
                    for(int j=0; j<JOINTS; j++) {
                        curveJoints[i].data.at(j) = d.getJointAngles()(j);
                    }
                }

                if(gripper) currentState = TO_FINAL;
                else if(goingHome) currentState = HOMING;
                else currentState = TO_BLOCK;

                // currentState = WAITING;

            break;

            case TO_BLOCK:
                std::cout << "\n[ToBlock] Moving to detected block\n";
                sleep(1);

                for(int i=0; i<noSteps; i++){
                    // float tmp = curveJoints[i].data.at(0);
                    // curveJoints[i].data.at(0) = curveJoints[i].data.at(2);
                    // curveJoints[i].data.at(2) = tmp;
                    jointPub.publish(curveJoints[i]);
                    jointStatus = false;
                    // requestPub.publish(msgJS);
                    while(!jointStatus) ros::spinOnce();
                }

                ee.computeDirect(q);
                std::cout << "Current position: " << ee.getPosition() << "\n\n";

                //attach dynamic link for block
                gripper = true;
                currentState = POSITION;

                // ev3d << -0.36646, -0.110244, -0.505492;
                // em3d << -0.441507, -0.693412, -0.569431,
                //         -0.826269, 0.0668009, 0.5593,
                //         -0.349787, 0.717439 , -0.602438;

                // ee.setPosition(ev3d);
                // ee.setOrientation(em3d);
                // d.computeInverse(ee);

                // for(int i=0; i<JOINTS; i++){
                //     curveJoints[0].data.at(i) = d.getJointAngles()[i];
                // }

                // std::cout << "\nHERE IT COMES\n";
                // sleep(3);
                // jointPub.publish(curveJoints[0]);

                currentState = WAITING;
            break;

            case TO_FINAL:
                std::cout << "\n[ToFinal] Moving to final destination\n";
                // for(int i=0; i<noSteps; i++){
                //     jointPub.publish(curveJoints[i]);
                //     jointStatus = false;
                //     requestPub.publish(msgJS);
                //     while(!jointStatus) ros::spinOnce();
                // }
                //detach dynamic link for block
                gripper = false;
                goingHome = true;
                currentState = POSITION;
            break;

            case HOMING:
                std::cout << "\n[Homing] going home\n";
                // for(int i=0; i<noSteps; i++){
                //     jointPub.publish(curveJoints[i]);
                //     jointStatus = false;
                //     requestPub.publish(msgJS);
                //     while(!jointStatus) ros::spinOnce();
                // }
                currentState = WAITING;
            break;

            default: std::cout << "\n[Error] invalid operation\n" << std::endl;
        }
    }
    #pragma endregion stateMachine

    return 0;
}