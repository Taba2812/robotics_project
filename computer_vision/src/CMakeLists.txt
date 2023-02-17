cmake_minimum_required(VERSION 2.8)
project( RobImgProPipe )

#OpenCV Stuff
find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )

#Custom Stuff
add_executable( ${PROJECT_NAME} image_processing_pipeline.cpp )

add_subdirectory(libraries)

target_link_libraries( ${PROJECT_NAME} ${OpenCV_LIBS} pipeline display_utility setting recognition temp_file_handler location_handler )