#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Float32MultiArray.h"
#include "libraries/bezier_curve.h"

#define RATIO 1000

int main (int argc, char **argv) {

    Bezier::Curve curve;
    Bezier::Node previous;

    //ROS Stuff
    ros::init(argc, argv, "MotionPlanning_Node");

    ros::NodeHandle handle;

    //Setup Params
    std::string pub_data, sub_data, sub_prog;
    int queue, resolution;
    handle.getParam("MP2Core_Data", pub_data);
    handle.getParam("Core2MP_Data", sub_data);
    handle.getParam("Core2MP_Req", sub_prog);
    handle.getParam("PATH_RESOLUTION", resolution);
    handle.getParam("Q_Size", queue);

    ros::Publisher data_publisher = handle.advertise<std_msgs::Float32MultiArray>(pub_data, queue);

    auto next_callback = [&] (const std_msgs::BoolConstPtr &next) {
        if (!(next->data)) {return;}

        Bezier::Node progress = curve.getNext();
        std_msgs::Float32MultiArray reply;
        std::cout << "[Motion Planning] Next point to reach -> x:" << progress.at(0) << " y:" << progress.at(1)<< " z:" << progress.at(2) << "\n";
        std::cout << "[Motion Planning] Previous point -> x:" << previous.at(0) << " y:" << previous.at(1)<< " z:" << previous.at(2) << "\n\n";
        reply.data = {progress[0] - previous[0], progress[1] - previous[1], progress[2] - previous[2]};
        previous = progress;
        data_publisher.publish(reply);
    };

    auto motion_planning_callback = [&] (const std_msgs::Float32MultiArrayConstPtr &destination) {
        std::cout << "[Motion Planning] Destination received as: x:" << (float)destination->data[0] << " y:" << (float)destination->data[1] << " z:" << (float)destination->data[2] << std::endl;
        Bezier::Node dest = {(float)destination->data[0],(float)destination->data[1],(float)destination->data[2]};
        previous = {0,0,0};

        curve = Bezier::Curve(dest, resolution, (int)destination->data[3], (int)destination->data[4]);

        if (destination->data[0] == 0.0f && destination->data[1] == 0.0f && destination->data[2] == 0.0f) {
            //Finished path, now reset curve waiting for new request
            curve = Bezier::Curve();
        } else {
            curve.getNext();
            Bezier::Node progress = curve.getNext();

            std_msgs::Float32MultiArray reply;
            reply.data.resize(3);

            std::cout << "[Motion Planning] Previous point -> x:" << previous.at(0) << " y:" << previous.at(1)<< " z:" << previous.at(2) << std::endl;
            std::cout << "[Motion Planning] Next point to reach -> x:" << progress.at(0) << " y:" << progress.at(1)<< " z:" << progress.at(2) << std::endl;
            
            reply.data = progress;
            previous = progress;

            std::cout << "[Motion Planning] Sending point -> x:" << progress.at(0) << " y:" << progress.at(1)<< " z:" << progress.at(2) << std::endl;
            data_publisher.publish(reply);
        }   
    };

    ros::Subscriber data_subscriber = handle.subscribe<std_msgs::Float32MultiArray>(sub_data, queue, motion_planning_callback);
    ros::Subscriber progress_subscriber = handle.subscribe<std_msgs::Bool>(sub_prog, queue, next_callback);

    ros::spin();

    return 0;
}