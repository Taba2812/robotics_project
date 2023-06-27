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

    int queue, noSteps;
    std::string to_gi, from_gi, to_detection, from_detection, to_motion_request, to_motion_data, from_motion;
    nh.getParam("Q_Size", queue);
    nh.getParam("UR52Gazebo", to_gi);
    nh.getParam("UR52Gazebo", from_gi);
    nh.getParam("Core2Det_Req", to_detection);
    nh.getParam("Det2Core_Res", from_detection);
    nh.getParam("Core2MP_Req", to_motion_request);
    nh.getParam("Core2MP_Data", to_motion_data);
    nh.getParam("MP2Core_Data", from_motion);
    nh.getParam("STEPS", noSteps);

    ros::Publisher pub_gi = nh.advertise<std_msgs::Float64MultiArray>(to_gi, queue);
    ros::Publisher pub_detection = nh.advertise<std_msgs::Bool>(to_detection, queue);
    ros::Publisher pub_motion_next = nh.advertise<std_msgs::Bool>(to_motion_request, queue);
    ros::Publisher pub_motion_data = nh.advertise<std_msgs::Float32MultiArray>(to_motion_data, queue);

    //CALLBACKS

    //Separare la classe Joint angles con una classe che sia un array a tre elementi per differenziarli
    ur5::JointAngles block_buffer, dest_buffer;
    bool block_buffer_empty = true;
    bool dest_buffer_empty = true;

    auto l_gi = [&] (const std_msgs::Float64MultiArrayConstPtr &position) {
        ur5::JointAngles received = gi.parseArray(position);

        if (block_buffer_empty && dest_buffer_empty) {
            //Send request to Detection
            std_msgs::Bool det_req;
            det_req.data = true;
            pub_detection.publish(det_req);
        } else {
            if (!block_buffer_empty) {
                //Se position == blocco
                    //block_buffer_empty = true;
                    //Invia richiesta al planning per il pat alla destination
                //Altrimenti
                    //Richiedere il prossimo step del percorso
                    //Inviarlo (fare inverse kinematic sul punto per inviare i joint);
            } else {
                //Se position == destination
                    //dest_buffer_empty = true;
                    //Continue... [Home, maybe in the future]
                //Altrimenti
                    //Richiede il prossimo step
                    //Inviarlo
            }
        }
    };

    auto l_detection = [&] (const std_msgs::Float32MultiArrayConstPtr &position) {
        block_buffer[0] = position->data[0];
        block_buffer[1] = position->data[1];
        block_buffer[2] = position->data[2];

        dest_buffer[0] = position->data[3];
        dest_buffer[1] = position->data[4];
        dest_buffer[2] = position->data[5];

        block_buffer_empty = false;
        dest_buffer_empty = false;

        pub_motion_data.publish(gi.createJointMessage32(block_buffer));
    };

    auto l_motion = [&] (const std_msgs::Float32MultiArrayConstPtr &position) {
        pub_gi.publish(position);
    };

    ros::Subscriber sub_gi = nh.subscribe<std_msgs::Float64MultiArray>(from_gi, queue, l_gi);
    ros::Subscriber sub_detection = nh.subscribe<std_msgs::Float32MultiArray>(from_detection, queue, l_detection);
    ros::Subscriber sub_motion = nh.subscribe<std_msgs::Float32MultiArray>(from_motion, queue, l_motion);

    ros::spin();

}