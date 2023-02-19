cmake_minimum_required(VERSION 2.8)
project( RobImgProPipe )

#OpenCV Stuff
find_package( OpenCV REQUIRED )
find_package(PCL 1.10 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  pcl_msgs
  pcl_conversions
  pcl_ros
)

include_directories( include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

link_directories(
  ${PCL_LIBRARY_DIRS}
)

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

catkin_install_python(PROGRAMS src/camera_rosrun.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

#Custom Stuff
add_subdirectory(libraries)

add_executable( test_sender test_true_send.cpp)

add_executable( ${PROJECT_NAME} image_processing_pipeline.cpp )
target_link_libraries( ${PROJECT_NAME} ${OpenCV_LIBS} pipeline display_utility setting recognition temp_file_handler location_handler detection )

add_executable( final block_detection_rosrun.cpp )
target_link_libraries( ${PROJECT_NAME} ${OpenCV_LIBS} ${PCL_LIBRARIES} pipeline display_utility setting recognition temp_file_handler location_handler detection )

install(TARGETS test_package
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)