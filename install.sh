#!/bin/bash

if [ ! -d "./opencv" ]; then
    git clone https://github.com/opencv/opencv.git
    mkdir -p opencv/build
    cd opencv/build

    cmake -DCMAKE_BUILD_TYPE=Release \
          -DCMAKE_INSTALL_PREFIX=../install \
          -DBUILD_LIST=core,imgcodecs \
          -DBUILD_EXAMPLES=OFF \
          -DBUILD_TESTS=OFF \
          -DBUILD_PERF_TESTS=OFF \
          -DBUILD_DOCS=OFF \
          -DWITH_IPP=OFF \
          -DWITH_TBB=OFF \
          -DWITH_OPENMP=OFF \
          -DWITH_CUDA=OFF \
          -DWITH_QT=OFF \
          -DBUILD_OPENJPEG:BOOL=ON \
          -DWITH_OPENGL=OFF ..

    cmake --build . --config Release -- -j$(nproc)
    cmake --install .
    cd ../../
fi

rm -f build/RRT_omp
rm -f build/RRT_pthread
rm -f build/RRT_serial
rm -f ./RRT_omp
rm -f ./RRT_pthread
rm -f ./RRT_serial

cmake -B build
cmake --build build

ln -s build/RRT_omp RRT_omp
ln -s build/RRT_pthread RRT_pthread
ln -s build/RRT_serial RRT_serial
