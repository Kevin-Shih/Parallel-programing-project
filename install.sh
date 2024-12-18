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

rm -f build/RRT
rm -f build/RRT_serial

cmake -B build
cmake --build build

ln -fs build/RRT RRT
ln -fs build/RRT_serial RRT_serial
