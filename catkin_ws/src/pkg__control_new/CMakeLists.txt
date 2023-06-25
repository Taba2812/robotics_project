cmake_minimum_required(VERSION 3.0.2)
project(pkg__control_new)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs sensor_msgs)
find_package(Eigen3 3.3 REQUIRED)

## Declare a catkin package
catkin_package()

add_subdirectory(src/libraries)

## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR})

set( OWN_LIB_SOURCES
  gazebo_interpreter
)

add_executable(UR5_Node src/ur5.node.cpp)
target_link_libraries(UR5_Node ${catkin_LIBRARIES} ${OWN_LIB_SOURCES})
install(TARGETS UR5_Node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)