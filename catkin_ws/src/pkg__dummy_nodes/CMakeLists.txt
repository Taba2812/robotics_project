cmake_minimum_required(VERSION 3.0.2)
project(pkg__dummy_nodes)

find_package(PCL 1.10 REQUIRED)
find_package(OpenCV 4.2.0 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  cv_bridge
  pcl_msgs
  pcl_conversions
  pcl_ros
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
  ${PCL_INCLUDE_DIRS}
)

link_directories(
  ${PCL_LIBRARY_DIRS}
)

add_executable(MotionPlanning_Dummy src/motion_planning.dummy.cpp)
target_link_libraries(MotionPlanning_Dummy ${catkin_LIBRARIES})

add_executable(Core-Detection_Dummy src/detection.dummy.cpp)
target_link_libraries(Core-Detection_Dummy ${catkin_LIBRARIES})

add_executable(CapturePlaceholders_Dummy src/save_camera_data.dummy.cpp)
target_link_libraries(CapturePlaceholders_Dummy ${catkin_LIBRARIES})

add_executable(SimulatedCapture_Dummy src/simulated_zed_capture.dummy.cpp)
target_link_libraries(SimulatedCapture_Dummy ${catkin_LIBRARIES})

add_executable(DisplayCamera_Dummy src/display_camera.dummy.cpp)
target_link_libraries(DisplayCamera_Dummy ${catkin_LIBRARIES})