#include "libraries/gazebo_interpreter.h"
#include "libraries/robot.h"
#include <ros/ros.h>
#include "std_msgs/Bool.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Float64MultiArray.h"

int main (int argc, char** argv) {
    ros::init(argc, argv, "UR5_Node");
    ros::NodeHandle nh;

    Gazebo::Interpreter gi;
    ur5::Robot robot;

    int queue, noSteps, frequency;
    std::string to_gi, from_gi, to_detection, from_detection, to_motion_request, to_motion_data, from_motion;
    nh.getParam("Q_Size", queue);
    nh.getParam("UR52Gazebo", to_gi);
    nh.getParam("Gazebo2UR5", from_gi);
    nh.getParam("Core2Det_Req", to_detection);
    nh.getParam("Det2Core_Res", from_detection);
    nh.getParam("Core2MP_Req", to_motion_request);
    nh.getParam("Core2MP_Data", to_motion_data);
    nh.getParam("MP2Core_Data", from_motion);
    nh.getParam("STEPS", noSteps);
    nh.getParam("FREQUENCY", frequency);

    ros::Rate rate(frequency);

    ros::Publisher pub_gi = nh.advertise<std_msgs::Float64MultiArray>(to_gi, queue);
    ros::Publisher pub_detection = nh.advertise<std_msgs::Bool>(to_detection, queue);
    ros::Publisher pub_motion_next = nh.advertise<std_msgs::Bool>(to_motion_request, queue);
    ros::Publisher pub_motion_data = nh.advertise<std_msgs::Float32MultiArray>(to_motion_data, queue);

    //CALLBACKS

    //Separare la classe Joint angles con una classe che sia un array a tre elementi per differenziarli
    Eigen::Vector3d block_buffer, dest_buffer;
    bool block_buffer_empty = true;
    bool dest_buffer_empty = true;

    auto l_send_to_gazebo_interface = [&] (ur5::JointAngles to_publish) {
        pub_gi.publish(gi.createJointMessage(to_publish));
    };

    auto l_send_to_motion_planning = [&] (const Eigen::Vector3d pos, const Eigen::Vector2d dir) {
        std::cout << "[ur5-Core] Sending path request to -Motion Planning-" << std::endl;
        std_msgs::Float32MultiArray block_coordinate;
        block_coordinate.data.resize(5);
        block_coordinate.data[0] = pos(0);
        block_coordinate.data[1] = pos(1);
        block_coordinate.data[2] = pos(2);
        block_coordinate.data[3] = dir(0);
        block_coordinate.data[4] = dir(1);

        sleep(1);
        pub_motion_data.publish(block_coordinate);
    };

    auto l_gi = [&] (const std_msgs::Float64MultiArrayConstPtr &position) {
        std::cout << "[ur5-Core] Received new JointState from -Gazebo Interface-" << std::endl;

        ur5::JointAngles received = gi.parseArray(position);
        robot.computeDirect(received);

        if (block_buffer_empty && dest_buffer_empty) {
            //Send request to Detection
            std_msgs::Bool det_req;
            det_req.data = true;
            pub_detection.publish(det_req);
        } else {
            std_msgs::Bool request_next_path_step;
            request_next_path_step.data = true;
            pub_motion_next.publish(request_next_path_step);

        }
    };

    auto l_detection = [&] (const std_msgs::Float32MultiArrayConstPtr &position) {
        std::cout << "[ur5-Core] Received block and destination positions from -Detection-" << std::endl;
        
        block_buffer(0) = position->data[0];
        block_buffer(1) = position->data[1];
        block_buffer(2) = position->data[2];

        dest_buffer(0) = position->data[3] - position->data[0];
        dest_buffer(1) = position->data[4] - position->data[1];
        dest_buffer(2) = position->data[5] - position->data[2];

        block_buffer_empty = false;
        dest_buffer_empty = false;

        l_send_to_motion_planning(block_buffer, Eigen::Vector2d(-1,1));
    };

    auto l_motion = [&] (const std_msgs::Float32MultiArrayConstPtr &step_coordinate) {
        
        Eigen::Vector3d translation(step_coordinate->data.at(0), step_coordinate->data.at(1), step_coordinate->data.at(2));
        if (translation != Eigen::Vector3d::Zero()) {
            ur5::Pose new_destination = robot.translateEndEffector(translation);

            ur5::Pose oriented(new_destination.getPosition(), Eigen::Vector3d(0,0,M_PI_2 + 1));

            ur5::JointAngles updated_joints = robot.computeInverse(oriented);

            std::cout << "[ur5-Core][Forwarding] Forwarding result of -Motion Planning-" << std::endl;
            
            //pub_gi.publish(gi.createJointMessage(updated_joints));
            l_send_to_gazebo_interface(updated_joints);
            // rate.sleep();

        } else {
            std::cout << "[ur5-Core] Arrived at destination" << std::endl;
            if (!block_buffer_empty && !dest_buffer_empty) {
                block_buffer_empty = true;

                l_send_to_motion_planning(dest_buffer, Eigen::Vector2d(1,1));
                // rate.sleep();

            } else if (block_buffer_empty) {
                dest_buffer_empty = true;
                std::cout << "[ur5-Core][Forwarding][3] Forwarding result of -Motion Planning-" << std::endl;

                //pub_gi.publish(gi.createJointMessage(robot.getHomeJoints()));
                sleep(2);
                l_send_to_gazebo_interface(robot.getHomeJoints());
                // rate.sleep();
            } 
        }
    };

    ros::Subscriber sub_gi = nh.subscribe<std_msgs::Float64MultiArray>(from_gi, queue, l_gi);
    ros::Subscriber sub_detection = nh.subscribe<std_msgs::Float32MultiArray>(from_detection, queue, l_detection);
    ros::Subscriber sub_motion = nh.subscribe<std_msgs::Float32MultiArray>(from_motion, queue, l_motion);

    ros::spin();

}