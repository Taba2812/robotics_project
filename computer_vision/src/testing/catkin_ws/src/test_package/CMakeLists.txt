cmake_minimum_required(VERSION 3.0.2)
project(test_package)

find_package(PCL 1.10 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  pcl_msgs
  pcl_conversions
  pcl_ros
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories( include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
)

link_directories(
  ${PCL_LIBRARY_DIRS}
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES 
  CATKIN_DEPENDS 
    roscpp 
    rospy 
    std_msgs 
    pcl_msgs
    pcl_conversions
    pcl_ros
  DEPENDS
    PCL
)

catkin_install_python(PROGRAMS src/msg.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})




add_executable(test_package src/receiver.cpp)
target_link_libraries( ${PROJECT_NAME} ${OpenCV_LIBS} ${catkin_LIBRARIES} ${roslib_LIBRARIES} ${PCL_LIBRARIES})
install(TARGETS test_package
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
