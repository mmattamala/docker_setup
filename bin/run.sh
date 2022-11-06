#!/bin/bash
# This script launches a WVN container for the target platform

set -e

# Source variables
source image_setup.sh

# Default target
TARGET=none

# Read arguments
for i in "$@"
do
case $i in
  -t=*|--target=*)
    TARGET=${i#*=}
    echo "[build.sh]: User-set target type is: '$TARGET'"
    shift
    ;;
esac
done


# Handle different target cases
if [[ "$TARGET" == "gpu" ]]; then
    docker run -it --rm --net=host --runtime nvidia -e DISPLAY=$DISPLAY -v /tmp/.X11-unix/:/tmp/.X11-unix -v ${GIT_WS}:/root/git -v ${CATKIN_WS}:/root/catkin_ws/ $GPU_IMAGE_TAG

elif [[ "$TARGET" == "cpu" ]]; then
    docker run -it --rm --net=host --gpus all --runtime nvidia -e DISPLAY=$DISPLAY -v /tmp/.X11-unix/:/tmp/.X11-unix -v ${GIT_WS}:/root/git -v ${CATKIN_WS}:/root/catkin_ws/ $CPU_IMAGE_TAG

else
    echo "Error: unsupported target [$TARGET]"
fi


