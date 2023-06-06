#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Float32MultiArray.h"

int main (int argc, char **argv) {
    ros::init(argc, argv, "MotionPlanning_Dummy");

    ros::NodeHandle handle;

    //setup params
    std::string pub_data, pub_req, sub_prog;
    int queue; 
    float x, y, z, step;
    handle.getParam("Core2MP_Data", pub_data);
    handle.getParam("Core2MP_Req", pub_req);
    handle.getParam("MP2Core_Data", sub_prog);
    handle.getParam("Q_Size", queue);
    handle.getParam("X", x);
    handle.getParam("Y", y);
    handle.getParam("Z", z);
    handle.getParam("STEP", step);

    ros::Publisher data_publisher = handle.advertise<std_msgs::Float32MultiArray>(pub_data, queue);
    ros::Publisher request_publisher = handle.advertise<std_msgs::Bool>(pub_req, queue);

    std_msgs::Float32MultiArray destination;
    destination.data = {x,y,z,step};

    int i = 0;
    auto progress_callback = [&] (const std_msgs::Float32MultiArrayConstPtr &next_position) {
        i++;
        //if (i > 30) {return;}

        std::cout << "[Core][Dummy] Step #" << i << " -> x:" << next_position->data[0] << " y: " << next_position->data[1] << " z: " << next_position->data[2] << std::endl;

        /*
        if (next_position->data[0] == destination.data[0] && next_position->data[1] == destination.data[1] && next_position->data[2] == destination.data[2]) {
            std::cout << "[Core][Dummy] Destinatio reached sending RESET signal" << std::endl;
            std_msgs::Float32MultiArray reset;
            reset.data = {0,0,0,0};
            data_publisher.publish(reset);
        } else {
            std_msgs::Bool next;
            next.data = true;
            request_publisher.publish(next);
        }
        */
        if (next_position->data[0] == 0 && next_position->data[1] == 0 && next_position->data[2] == 0) {
            std::cout << "[Core][Dummy] Destinatio reached sending RESET signal" << std::endl;
            std_msgs::Float32MultiArray reset;
            reset.data = {0,0,0,0};
            data_publisher.publish(reset);
        } else {
            std_msgs::Bool next;
            next.data = true;
            request_publisher.publish(next);
        }
        
    };

    ros::Subscriber data_subscriber = handle.subscribe<std_msgs::Float32MultiArray>(sub_prog, queue, progress_callback);

    std::cout << "[Core][Dummy] Press any key to send a request the path segments: ";
    std::cin.get();

    std::cout << "[Core][Dummy] Requesting path to [" << x << "," << y << "," << z << "] in " << step << " steps" << std::endl;
    data_publisher.publish(destination);

    ros::spin();

    return 0;
}