#!/bin/bash
set -e

# Read arguments
for i in "$@"
do
case $i in
  -t=*|--ros-version=*)
    ROS_VERSION=${i#*=}
    shift
    ;;
  -t=*|--cuda=*)
    CUDA=${i#*=}
    shift
    ;;
  -t=*|--cuda-arch-bin=*)
    CUDA_ARCH_BIN=${i#*=}
    shift
    ;;
esac
done

#
# Install OpenCV
#
apt-get update
apt-get install -y --no-install-recommends \
        libopencv-dev=4.2.0+dfsg-5 \
        python3-opencv
# Remove cache
rm -rf /var/lib/apt/lists/*

if [[ "$CUDA" == "true" ]]; then
    echo "Building OpenCV compatible with ROS $ROS_VERSION"

    if [[ "$ROS_VERSION" == "noetic" ]]; then
        OPENCV_VERSION="4.2.0"
    elif [[ "$ROS_VERSION" == "melodic" ]]; then
        OPENCV_VERSION="3.4.16"
    else
        echo "Error: unsupported ROS version $ROS_VERSION"
    fi

    # Install OpenCV
    echo $PWD
    git clone --depth 1 --branch ${OPENCV_VERSION} https://github.com/opencv/opencv.git && \
    git clone --depth 1 --branch ${OPENCV_VERSION} https://github.com/opencv/opencv_contrib.git && \
    mkdir /opencv/build
    cd /opencv/build
    echo $PWD
    echo "Configuring OpenCV ${OPENCV_VERSION}, CUDA_ARCH_BIN=${CUDA_ARCH_BIN}"
    cmake \
        -D CPACK_BINARY_DEB=ON \
        -D CUDA_nppicom_LIBRARY="" \
        -D BUILD_EXAMPLES=OFF \
        -D BUILD_opencv_python2=OFF \
        -D BUILD_opencv_python3=ON \
        -D BUILD_opencv_java=OFF \
        -D CMAKE_BUILD_TYPE=RELEASE \
        -D CMAKE_INSTALL_PREFIX=/usr/local \
        -D CUDA_ARCH_BIN=${CUDA_ARCH_BIN} \
        -D CUDA_ARCH_PTX= \
        -D CUDA_FAST_MATH=ON \
        -D CUDNN_INCLUDE_DIR=/usr/include/$(uname -i)-linux-gnu \
        -D EIGEN_INCLUDE_PATH=/usr/include/eigen3 \
        -D WITH_EIGEN=ON \
        -D ENABLE_NEON=OFF \
        -D OPENCV_DNN_CUDA=OFF \
        -D OPENCV_ENABLE_NONFREE=ON \
        -D OPENCV_EXTRA_MODULES_PATH=/opencv_contrib/modules \
        -D OPENCV_GENERATE_PKGCONFIG=ON \
        -D WITH_CUBLAS=ON \
        -D WITH_CUDA=ON \
        -D WITH_CUDNN=ON \
        -D CUDNN_VERSION=8.3 \
        -D CUDNN_INCLUDE_DIR=/usr/include \
        -D WITH_GSTREAMER=ON \
        -D WITH_LIBV4L=ON \
        -D WITH_OPENGL=ON \
        -D WITH_OPENCL=ON \
        -D WITH_IPP=OFF \
        -D WITH_TBB=ON \
        -D BUILD_TIFF=ON \
        -D BUILD_PERF_TESTS=OFF \
        -D BUILD_TESTS=OFF \
        -D BUILD_opencv_cudaarithm=ON \
        -D BUILD_opencv_cudafilters=ON \
        -D BUILD_opencv_cudalegacy=ON \
        -D BUILD_opencv_cudev=ON \
        -D BUILD_opencv_xfeatures2d=ON \
        -D BUILD_opencv_ximgproc=ON \
        -D BUILD_opencv_nonfree=ON \
        -D BUILD_opencv_xphoto=ON \
        ../
    
    cd /opencv/build && echo $PWD && make -j$(nproc)
    echo $PWD
    make install
    make package
    cd /
    echo $PWD

    # Remove OpenCv folder
    rm -rf opencv && rm -rf opencv_contrib

else
    echo "No CUDA required, skipping"
fi