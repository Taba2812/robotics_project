cmake_minimum_required(VERSION 3.0.2)
project(pkg__dummy_nodes)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  pcl_msgs
  pcl_conversions
  pcl_ros
)

add_subdirectory(../shared_libraries ${CMAKE_CURRENT_BINARY_DIR}/libraries)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES pkg__dummy_nodes
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_executable(MotionPlanning_Dummy src/motion_planning.dummy.cpp)
target_link_libraries(MotionPlanning_Dummy ${catkin_LIBRARIES})

add_executable(Detection_Dummy src/detection.dummy.cpp)
target_link_libraries(Detection_Dummy ${catkin_LIBRARIES})

add_executable(Camera_Dummy src/placeholder_camera_data.dummy.cpp)
target_link_libraries(Camera_Dummy ${catkin_LIBRARIES} temp_file_handler)
