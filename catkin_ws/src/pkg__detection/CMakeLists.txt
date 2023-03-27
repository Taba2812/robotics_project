cmake_minimum_required(VERSION 3.0.2)
project(pkg__detection)

find_package(OpenCV REQUIRED)
find_package(PCL 1.10 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  pcl_msgs
  pcl_conversions
  pcl_ros
  cv_bridge
)

catkin_package(
)

include_directories( include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_subdirectory(src/libraries)

link_directories(
  ${PCL_LIBRARY_DIRS}
)

add_executable(Detector_Node src/block_detection_rosrun.cpp)
target_link_libraries(Detector_Node ${catkin_LIBRARIES} detection location_handler temp_file_handler recognition pipeline setting display_utility)
install(TARGETS Detector_Node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
