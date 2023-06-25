find_package(Eigen3 3.3 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
)


include_directories( include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

add_library(gazebo_interpreter gazebo_interpreter.cpp)
add_library(robot robot.cpp)

# target_link_libraries(data_type_handler ${catkin_LIBRARIES})