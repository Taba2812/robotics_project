#include "libraries/gazebo_interpreter.h"
#include <ros/ros.h>
#include "std_msgs/Bool.h"

int main (int argc, char** argv) {
    ros::init(argc, argv, "UR5_Node");
    ros::NodeHandle nh;

    int queue_size, noSteps;
    std::string joint_in, joint_out, joint_req, motion_req, motion_data, motion_res;
    nh.getParam("Q_Size", queue_size);
    nh.getParam("Core2JS_Req", joint_req);
    nh.getParam("Core2JS_Data", joint_out);
    nh.getParam("JS2Core_Data", joint_in);
    // nh.getParam("Docker_Joint_In", joint_in);
    // nh.getParam("Docker_Joint_Out", joint_out);
    nh.getParam("Core2MP_Req", motion_req);
    nh.getParam("Core2MP_Data", motion_data);
    nh.getParam("MP2Core_Data", motion_res);
    nh.getParam("STEPS", noSteps);

    Gazebo::Interpreter gi;
    ur5::JointAngles angles;

    ros::Publisher jointPub = nh.advertise<sensor_msgs::JointState>(joint_out, queue_size);
    ros::Publisher jointReq = nh.advertise<std_msgs::Bool>(joint_req, queue_size);
    ros::Publisher motionPub = nh.advertise<std_msgs::Bool>(motion_req, queue_size);
    ros::Publisher dataPub = nh.advertise<std_msgs::Float32MultiArray>(motion_data, queue_size);

    bool jointStatus = false;
    auto getJoint = [&] (const sensor_msgs::JointState::ConstPtr &js) {
        if (jointStatus) { return; }

        angles = gi.parseJointState(js);

        jointStatus = true;
    };

    ur5::Position curveSteps[noSteps];
    std_msgs::Float32MultiArray msgMotion;
    bool motionStatus = false;
    int motionCounter = 0;
    auto getMotion = [&] (const std_msgs::Float32MultiArrayConstPtr &next_position) {

        if(motionStatus) { return; }

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
            } else {
                motionCounter = 0;
                motionStatus = true;
            }
        }
    };

    ros::Subscriber jointSub = nh.subscribe<sensor_msgs::JointState>(joint_in, queue_size, getJoint);
    ros::Subscriber motionSub = nh.subscribe<std_msgs::Float32MultiArray>(motion_res, queue_size, getMotion);

    // int index;
    // double angle;
    char c;

    ur5::Robot robot;
    std_msgs::Bool jointMsg;

    // // end effector
    // const double eeX = 0.151843;
    // const double eeY = 0.190801;
    // const double eeZ = -0.454981;
    // const ur5::Position eePosition(eeX, eeY, eeZ);

    // const double eeRoll = 2.899031;
    // const double eePitch = -0.087728;
    // const double eeYaw = 1.553114;
    // const ur5::EulerAngles eeRotation(eeRoll, eePitch, eeYaw);
    // const ur5::Orientation eeOrientation = robot.computeOrientation(eeRotation);

    // ur5::JointAngles eeJointAngles;

    // // block
    // const double blockX = 0.64;
    // const double blockY = 0.6;
    // const double blockZ = -0.825;
    // const ur5::Position blockPosition(blockX, blockY, blockZ);

    // const double blockRoll = 0;
    // const double blockPitch = 0;
    // const double blockYaw = 1;
    // const ur5::EulerAngles blockRotation(blockRoll, blockPitch, blockYaw);
    // const ur5::Orientation blockOrientation = robot.computeOrientation(blockRotation);

    // ur5::JointAngles blockJointAngles;

    // ur5::JointAngles interpolatedJointAngles[noSteps];

    while(ros::ok()) {  
        std::cout << "Continue: ";
        std::cin >> c;

        jointMsg.data = true;
        jointReq.publish(jointMsg);

        while(!jointStatus) ros::spinOnce();
        jointStatus = false;

        robot.computeDirect(angles);
        std::cout << robot.getPose() << "\n\n";

        ur5::JointAngles ja = robot.computeInverse(robot.getPose());
        std::cout << "\nJoint angles: " << ja << "\n";
        
        // while(!jointStatus) ros::spinOnce();
        // jointStatus = false;

        // ur5::JointAngles ja = gi.getJointAngles();
        // robot.computeDirect(ja);

        // std::cout << "Position: " << robot.getPosition() << "\n\n";
        // std::cout << "Orientation: " << robot.getOrientation() << "\n\n";

        // std::cout << "Which joint do you want to move? ";
        // std::cin >> index;

        // std::cout << "How much? ";
        // std::cin >> angle;

        // ur5::JointAngles ja = angles;
        // ja(index) += M_PI / angle;

        // Gazebo::JointMessage jm = gi.createJointMessage(ja);

        // jointPub.publish(jm);
    }
}