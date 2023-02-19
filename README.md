# robotics_project
Project on a robotic manipulator.

git clone git@github.com:Taba2812/robotics_project.git
git submodule update --recursive

# Working Components

The working components are gathered in separate packages inside of the 'catkin_ws/src' directory.

## Object Detection

This material is inside the `pkg__detection` package. It contains 3 executables:
  * `block_detection_rosrun.cpp`: this file contains the final ros node that handles object detection requests from the main nodes, this ode depends on the installation of an external library to handle raw memory conversion in the recieving of point cloud data. [pcl_ROS](http://wiki.ros.org/pcl_ros) is the library in question and installation instruction can be found [here](https://github.com/methylDragon/pcl-ros-tutorial/blob/master/PCL%20Reference%20with%20ROS.md);
  * `test_true_send.cpp`: this file contains a test rosnode to send object detection requests to the object detection node, without initializing the rest of the ros setup;
  * `image_processing_pipeline.cpp`: this file contains a demonstation of object detection with OpenCV windows to look into the object detection process. This executable can be built individually using CMake directly, a connection to the catkin system has not yet been implemented for this executable;
  
The database for all images and templates used by this code can be found in the 'image_database' directory, it contains the following:
  * `blocks_dataset`: this directory contains all the templates utilized in the implementation of OpenCV's [generalizedHughGuil](https://github.com/Taba2812/robotics_project/blob/main/catkin_ws/src/pkg__detection/src/libraries/recognition.cpp);
  * `complete_data_examples`: this directory contains 2 sets of complete Zed2 Data comprehensive of .png image and Point Cloud cv::Mat saved as a binary file;
  * `official_placeholders`: this directory contains images provided by the professors of multiple megablocks in different colors on the testing table;
  * `testing_placeholders`: this directory contains images I ([Davide Mongera](https://github.com/Dawwo20415)) took of the testing table with a variety of megablocks;
  
## Zed2 Controller

This component is stored in the `pkg__zed2_camera` package, and contains the following, this code is written in python due to compatibility issues with the c++ Zed2 sdk and our installation of the Nvidia CUDA drivers necessary for it to function:
  * `camera_rosrun.py`: which is the main node that handles interacting with the [Zed2](https://www.stereolabs.com/zed-2/) camera;
  * `save_instance_to_file.py`: is a node that connects to the Zed2 camera and saves the data to binary files for later use; 

## Position Control
Code stored inside the `pkg__position_control/src` directory
  * `headers`: this directory contains the libraries concerning position control (direct and inverse kinematics, ecc.)
  * `state machine`: this directory contains the files concerning the management of the state machine
  * `main.cpp`: main file of position control, manages the state machine while communicating with computer vision and (theoretically) motion control
  * `foo.cpp`: not necessary, it does contain an implementation of inverse kinematics detached from the project
