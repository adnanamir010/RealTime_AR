cmake_minimum_required(VERSION 3.27)
project(Project4)

set(CMAKE_CXX_STANDARD 17)
set(ENV{OPENCV_DIR} "C:\\lib\\install\\opencv")
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/")

find_package(OpenCV REQUIRED)

include_directories(include,
        ${OpenCV_INCLUDE_DIRS}
)

add_executable(Project4 src/main.cpp
        src/utils.cpp
        include/utils.h

)

set(OpenCV_LIBS opencv_core opencv_imgproc opencv_highgui opencv_imgcodecs opencv_aruco)
target_link_libraries(Project4 ${OpenCV_LIBS})