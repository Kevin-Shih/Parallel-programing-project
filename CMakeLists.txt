cmake_minimum_required(VERSION 3.10)
project(RRT_Parallel_Project)

set(OpenCV_DIR "${CMAKE_SOURCE_DIR}/opencv/install/lib/cmake/opencv4")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -O3")

find_package(OpenCV REQUIRED core imgcodecs)

include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(RRT
    src/RRT.cpp
    src/Util.cpp)

target_link_libraries(RRT ${OpenCV_LIBS})
