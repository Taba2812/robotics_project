cmake_minimum_required(VERSION 2.8)
project( RobImgProPipe )

#OpenCV Stuff
find_package( OpenCV REQUIRED )
find_package(PCL 1.10 REQUIRED)
include_directories( ${OpenCV_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS} )

link_directories(
  ${PCL_LIBRARY_DIRS}
)

#Custom Stuff
add_subdirectory(libraries)

add_executable( ${PROJECT_NAME} image_processing_pipeline.cpp )
target_link_libraries( ${PROJECT_NAME} ${OpenCV_LIBS} pipeline display_utility setting recognition temp_file_handler location_handler )

add_executable( final block_detection_rosrun.cpp )
target_link_libraries( ${PROJECT_NAME} ${OpenCV_LIBS} ${PCL_LIBRARIES} pipeline display_utility setting recognition temp_file_handler location_handler )