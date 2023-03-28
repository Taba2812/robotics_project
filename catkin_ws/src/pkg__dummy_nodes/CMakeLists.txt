cmake_minimum_required(VERSION 3.0.2)
project(pkg__dummy_nodes)

find_package(OpenCV 4.2.0 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  cv_bridge
)

add_subdirectory(src/libraries)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES pkg__dummy_nodes
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

include_directories( include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(MotionPlanning_Dummy src/motion_planning.dummy.cpp)
target_link_libraries(MotionPlanning_Dummy ${catkin_LIBRARIES})

add_executable(Detection_Dummy src/detection.dummy.cpp)
target_link_libraries(Detection_Dummy ${catkin_LIBRARIES})

catkin_install_python(PROGRAMS src/placeholder_camera_data.dummy.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})