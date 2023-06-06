#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "std_msgs/Bool.h"

#define JOINTS 6
#define LOOP_RATE 100


int main(int argc, char** argv) {
  ros::init(argc, argv, "JointStates_Dummy");
  ros::NodeHandle nh;
  ros::Rate loopRate(LOOP_RATE);

  sensor_msgs::JointState jointStateMsg;
  jointStateMsg.name = {"elbow_joint", "shoulder_lift_joint", "shoulder_pan_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"};
  jointStateMsg.position = {0.0, 0.5, 1.0, 1.5, 2.0, 2.5};

  std::string js_data, js_new, js_req;
  int queue_size;
  nh.getParam("JS2Core_Data", js_data);
  nh.getParam("Q_Size", queue_size);
  nh.getParam("Core2JS_Data", js_new);
  nh.getParam("Core2JS_Req", js_req);

  ros::Publisher jointStatePub = nh.advertise<sensor_msgs::JointState>(js_data, queue_size);

  auto jointStateCallback = [&] (const sensor_msgs::JointState::ConstPtr &msg) {
    std::cout << "\n[JointStates] received new values\n";

    for(int i=0; i<JOINTS; i++){
      jointStateMsg.position[i] = msg->position[i];
    }
  };

  auto requestCallback = [&] (const std_msgs::Bool::ConstPtr &req) {
    std::cout << "[Joint States] Received request - sending home joint states:\n";
    for(int i=0; i<JOINTS; i++){
      std::cout << jointStateMsg.position.at(i) << "\n";
    }
    jointStatePub.publish(jointStateMsg);
  };

  ros::Subscriber jointStateSub = nh.subscribe<sensor_msgs::JointState>(js_new, queue_size, jointStateCallback);
  ros::Subscriber requestSub = nh.subscribe<std_msgs::Bool>(js_req, queue_size, requestCallback);

  ros::spin();

  return 0;
}
