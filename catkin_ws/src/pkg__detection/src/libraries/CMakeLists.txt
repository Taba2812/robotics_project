find_package(OpenCV 4.2.0 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  pcl_msgs
  pcl_conversions
  pcl_ros
  cv_bridge
)


include_directories( include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

link_directories(
  ${PCL_LIBRARY_DIRS}
)

add_library(setting settings.cpp)
add_library(pipeline pipeline.cpp)
add_library(display_utility display_utility.cpp)
add_library(recognition recognition.cpp)
add_library(location_handler location_handler.cpp)
add_library(detection detection.cpp)
add_library(temp_file_handler_2 temp_file_handler.cpp)
add_library(data_type_handler data_type_handler.cpp)
target_link_libraries(data_type_handler ${catkin_LIBRARIES})