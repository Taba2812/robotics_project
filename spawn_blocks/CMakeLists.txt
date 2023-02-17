cmake_minimum_required(VERSION 2.8.3)
project(spawn_blocks)

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs gazebo_ros)

## Declare a catkin package
catkin_package()

## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(spawner src/spawn.cpp)
target_link_libraries(spawner ${catkin_LIBRARIES})
install(TARGETS spawner
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)