cmake_minimum_required(VERSION 3.10)
project(RRT_Parallel_Project LANGUAGES CUDA CXX)

set(OpenCV_DIR "${CMAKE_SOURCE_DIR}/opencv/install/lib/cmake/opencv4")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}  -O3 -std=c++17 -Wall -g -fPIC")
set(CMAKE_CUDA_FLAGS "${CMAKE_CUDA_FLAGS} --device-c -gencode=arch=compute_61,code=sm_61 -Xcompiler '-fPIC' -g -O3")

find_package(OpenCV REQUIRED core imgcodecs)

include_directories(${OpenCV_INCLUDE_DIRS})
# include_directories(${CUDA_INCLUDE_DIRS})

add_executable(RRT_cuda
    src/RRT_cuda.cpp
    src/kernel_util.cu)

target_link_libraries(RRT_cuda ${OpenCV_LIBS})
set_target_properties(RRT_cuda PROPERTIES CUDA_ARCHITECTURES "61")
