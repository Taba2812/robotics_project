cmake_minimum_required(VERSION 3.0.2)
project(pkg__motion_planning)

find_package(catkin REQUIRED)


catkin_package(
)

include_directories(
)

add_subdirectory(src/libraries)

add_executable(MotionPlanning_Node src/motion_planning_node.cpp)
target_link_libraries(MotionPlanning_Node ${catkin_LIBRARIES} bezier_curve motion_planning)
