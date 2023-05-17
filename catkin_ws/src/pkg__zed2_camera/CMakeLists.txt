cmake_minimum_required(VERSION 3.0.2)
project(pkg__zed2_camera)

find_package(catkin REQUIRED COMPONENTS
  pcl_conversions
  pcl_ros
  pcl_msgs
  rospy
  sensor_msgs
  std_msgs
)

catkin_package()

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(Docker_Proxy src/docker_proxy.node.cpp)
target_link_libraries(Docker_Proxy ${catkin_LIBRARIES})

#catkin_install_python(PROGRAMS src/camera_rosrun.py
#  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)