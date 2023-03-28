find_package(OpenCV 4.2.0 REQUIRED)

include_directories( include
    ${OpenCV_INCLUDE_DIRS}    
)

add_library(temp_file_handler temp_file_handler.cpp)
target_link_libraries(temp_file_handler ${OpenCV_LIBS})