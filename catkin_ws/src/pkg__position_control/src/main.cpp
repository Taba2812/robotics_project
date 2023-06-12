#include "headers/direct_kinematics.h"
#include "headers/inverse_kinematics.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Float64.h"

int main(int argc, char **argv){
    sleep(1);

    #pragma region environment

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
    nh.getParam("Steps", noSteps);
    nh.getParam("Q_Size", queue_size);

    //environment components
    EndEffector ee;
    Destination d, home;
    FinalDestination fd;
    Eigen::Vector3d initialStep;
    Eigen::Vector3d finalStep;
    Eigen::Vector3d currentPosition;
    BlockPosition bp;

    //ros messages
    std_msgs::Bool msgVision;
    std_msgs::Bool msgJS;
    std_msgs::Float32MultiArray msgMotion;

    //default values
    bool jointStatus = false;
    bool positionStatus = false;
    bool motionStatus = false;
    bool gripper = false;
    bool defaultPosition = true;

    bp = BlockPosition::Zero();
    msgMotion.data = {0,0,0,0};
    msgJS.data = true;

    //user interaction
    char c;

    // curve generation steps
    Eigen::Vector3d curveSteps[noSteps];
    Eigen::Vector3d curvePositions[noSteps];
    std_msgs::Float32MultiArray curveJoints[noSteps];

    //publishers
    //ros::Publisher jointPub = nh.advertise<std_msgs::Float64MultiArray>(joint_docker_out, QUEUE_SIZE);
    ros::Publisher jointPub = nh.advertise<sensor_msgs::JointState>(js_new, queue_size);
    ros::Publisher visionPub = nh.advertise<std_msgs::Bool>(detection_req, queue_size);
    ros::Publisher motionPub = nh.advertise<std_msgs::Bool>(motion_req, queue_size);
    ros::Publisher dataPub = nh.advertise<std_msgs::Float32MultiArray>(motion_data, queue_size);
    ros::Publisher requestPub = nh.advertise<std_msgs::Bool>(js_req, queue_size);

    #pragma endregion environment

    #pragma region callbacks
    //lambda callbacks
    int motionCounter = 0;
    auto getMotion = [&] (const std_msgs::Float32MultiArrayConstPtr &next_position) {

        std::cout << "[" << motionCounter << "] ";    
        for(int i=0; i<3; i++) std::cout << next_position->data.at(i) << " ";
        std::cout << std::endl;

        if (next_position->data.at(0) == d.getPosition()[0] &&
            next_position->data.at(1) == d.getPosition()[1] &&
            next_position->data.at(2) == d.getPosition()[2]) {

            motionCounter = 0;

            std::cout << "\n[Core] Destination reached sending RESET signal\n";
            std_msgs::Float32MultiArray reset;
            reset.data = {0,0,0,0};
            dataPub.publish(reset);

        } else {

            for(int i=0; i<3; i++){
                curveSteps[motionCounter](i) = next_position->data.at(i);
            }

            std_msgs::Bool next;
            next.data = true;
            motionPub.publish(next);
        }

        motionCounter++;

        if(motionCounter == noSteps) motionStatus = true;
    };

    auto getJoint = [&] (const sensor_msgs::JointState::ConstPtr &js) {
        std::cout << "[Main] received joint states\n";
        for(int i=0; i<JOINTS; i++){
            for(int j=0; j<JOINTS; j++){
                if(jointNames[j] == js->name.at(i)){
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

        bp(0) = xyz->data[0];
        bp(1) = xyz->data[1];
        bp(2) = xyz->data[2];

        for(int i=0; i<3; i++){
            std::cout << "bp(" << i << ") = " << bp(i) << "\n";
        }

        positionStatus = true;
    };

    //subscribers
    ros::Subscriber motionSub = nh.subscribe<std_msgs::Float32MultiArray>(motion_res, queue_size, getMotion);
    //ros::Subscriber jointSub = nh.subscribe<sensor_msgs::JointState>(joint_docker_in, queue_size, getJoint);
    ros::Subscriber jointSub = nh.subscribe<sensor_msgs::JointState>(js_data, queue_size, getJoint);
    ros::Subscriber visionSub = nh.subscribe<std_msgs::Float32MultiArray>(detection_res, queue_size, getPosition);

    #pragma endregion callbacks
    
    int currentState = WAITING;

    #pragma region stateMachine
    while(ros::ok()){
        switch(currentState){

            case WAITING:
                std::cout << "\n";
                std::cout << "[Waiting] press any key to advance: ";
                std::cin >> c;

                currentState = JS;
            break;

            case JS:
                std::cout << "\n[JointStates] retrieving joint states\n";

                jointStatus = false;
                requestPub.publish(msgJS);

                while(!jointStatus) ros::spinOnce();
                ee.computeDirect(q);

                if(defaultPosition){
                    home.setPosition(ee.getPosition());
                    defaultPosition = false;
                }

                std::cout << "\nHome: \n" << home.getPosition() << "\n";
                
                currentState = VISION;
                msgVision.data = true;
                visionPub.publish(msgVision);
            break;

            case VISION:
                std::cout << "\n[Vision] Waiting for detection\n";

                while(!positionStatus) ros::spinOnce();
                currentState = POSITION;
            break;

            case POSITION:
                std::cout << "\n[Position] Evaluating the destination...";
                if(!gripper){
                    std::cout << "going from home to block\n";
                    d.setPosition(bp);
                } else {
                    std::cout << "going from block to destination\n";
                    d.setPosition(fd);
                }
                d.computeInverse(ee);
                std::cout << "\nDestination: \n" << d.getJointAngles() << "\n";

                if(d.isWithinJointLimits(d.getJointAngles())) std::cout << "\n[Position] Destination reachable!\n";
                else std::cout << "\n[Error] Destination not reachable!\n";

                currentState = MOTION;
            break;

            case MOTION:
                std::cout << "\n[Motion] computing path planning...\n";
                initialStep << ee.getPosition();
                finalStep << d.getPosition();

                for(int i=0; i<3; i++){
                    msgMotion.data.at(i) = finalStep(i);
                }

                msgMotion.data.at(3) = noSteps;
                
                dataPub.publish(msgMotion);

                while(!motionStatus) ros::spinOnce();

                currentPosition << initialStep;

                for(int i=0; i<noSteps; i++) {
                    curvePositions[i] << currentPosition + curveSteps[i];
                    currentPosition << curvePositions[i];

                    ee.setPosition(curvePositions[i]);
                    d.computeInverse(ee);
                    
                    curveJoints[i].data.resize(JOINTS);
                    for(int j=0; j<JOINTS; j++) {
                        curveJoints[i].data.at(j) = d.getJointAngles()(j);
                    }
                }

                if(!gripper) currentState = TO_BLOCK;
                else currentState = TO_FINAL;

                currentState = WAITING;
            break;

            case TO_BLOCK:
                std::cout << "\n[ToBlock] Moving to detected block\n";
                for(int i=0; i<noSteps; i++){
                    jointPub.publish(curveJoints[i]);
                }
                //attach dynamic link for block
                gripper = true;
                currentState = POSITION;
            break;

            case TO_FINAL:
                std::cout << "\n[ToFinal] Moving to final destination\n";
                for(int i=0; i<noSteps; i++){
                    jointPub.publish(d.getMessage());
                }
                //detach dynamic link for block
                gripper = false;
                currentState = HOMING;
            break;

            case HOMING:
                std::cout << "\n[Homing] going home\n";
                //publish topic
                while(!processStatus) usleep(WAITING_TIME);
                for(int i=0; i<noSteps; i++){
                    jointPub.publish(home.getMessage());
                }
                currentState = VISION;
            break;

            default: std::cout << "\n[Error] invalid operation\n" << std::endl;
        }
    }
    #pragma endregion stateMachine

    return 0;
}