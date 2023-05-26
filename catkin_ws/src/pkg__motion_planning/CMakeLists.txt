cmake_minimum_required(VERSION 3.0.2)
project(pkg__motion_planning)

find_package(catkin REQUIRED COMPONENTS
    roscpp
)


catkin_package(
)

include_directories( include 
    ${catkin_INCLUDE_DIRS}
)

add_subdirectory(src/libraries)

add_executable(MotionPlanning_Node src/motion_planning.node.cpp)
target_link_libraries(MotionPlanning_Node ${catkin_LIBRARIES} bezier_curve motion_planning)

add_executable(MotionPlanning_Test src/motion_planning.test.cpp)
target_link_libraries(MotionPlanning_Test ${catkin_LIBRARIES} bezier_curve motion_planning)
