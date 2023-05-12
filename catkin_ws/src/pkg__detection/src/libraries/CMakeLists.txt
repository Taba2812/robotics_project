find_package(OpenCV 4.2.0 REQUIRED)

include_directories( include
    ${OpenCV_INCLUDE_DIRS}    
)

add_library(setting settings.cpp)
add_library(pipeline pipeline.cpp)
add_library(display_utility display_utility.cpp)
add_library(recognition recognition.cpp)
add_library(location_handler location_handler.cpp)
add_library(detection detection.cpp)
add_library(temp_file_handler_2 temp_file_handler.cpp)
add_library(data_type_handler data_type_handler.cpp)
target_link_libraries(data_type_handler ${OpenCV_LIBS})