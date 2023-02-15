#include <iostream>
#include <cstdlib>
#include <ctime>
#include "gazebo_msgs/SpawnModel.h"
#include "geometry_msgs/Pose.h"
#include "ros/ros.h"
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/Int16.h"

#define Z 0.9
#define LOOP_RATE 1
#define MODELS_NUM 11
#define MAX_X 5
#define MAX_Y 5

const std::string path = "~/trento_lab_home/ros_ws/src/spawn_blocks/models/";
const std::string file = "/model.sdf";
std::string models[MODELS_NUM] = {"X1-Y1-Z2", "X1-Y2-Z1", "X1-Y2-Z2", "X1-Y2-Z2-CHAMFER", "X1-Y2-Z2-TWINFILLER",
                                  "X1-Y3-Z2", "X1-Y3-Z2-FILLET", "X1-Y4-Z1", "X1-Y4-Z2", "X2-Y2-Z2", "X2-Y2-Z2-FILLET"};

std::string random_block(){
    int block_index = rand() % MODELS_NUM + 1;
    return models[block_index];
}

double random_value(int max){
    double value = (rand() / (double)max) * max;
    return value;
}

int main(int argc, char **argv){
    //ros::init(argc, argv, "spawner");
    //ros::NodeHandle nh;
    //ros::Rate loop_rate(LOOP_RATE);

    //ros::Publisher block_pub = nh.subscribe()

    srand(time(NULL));

    std::string model = path + random_block() + file;
    double X = random_value(MAX_X);
    double Y = random_value(MAX_Y);

    return 0;
}