#!/bin/bash
# This script launches a container for the target platform

set -e

# Default target
TARGET=none
GIT_DIR="$HOME/git"
CATKIN_DIR="$HOME/catkin_ws"

# Read arguments
for i in "$@"
do
case $i in
    -t=*|--target=*)
        TARGET=${i#*=}
        echo "[run.sh]: User-set target type is: '$TARGET'"
        shift
        ;;
    -i=*|--image-id=*)
        IMAGE_ID=${i#*=}
        echo "[run.sh]: User-set IMAGE_ID is: '$IMAGE_ID'"
        shift
        ;;
    -g=*|--git-dir=*)
        GIT_DIR=${i#*=}
        echo "[run.sh]: User-set GIT_DIR is: '$GIT_DIR'"
        shift
        ;;
    -c=*|--catkin-dir=*)
        CATKIN_DIR=${i#*=}
        echo "[run.sh]: User-set CATKIN_DIR is: '$CATKIN_DIR'"
        shift
        ;;
esac
done


# Handle different target cases
source targets/$TARGET.sh

# Run docker
if [[ "$TARGET" != "none" ]]; then
    docker run -it --rm --net=host --runtime nvidia -e DISPLAY=$DISPLAY -v /tmp/.X11-unix/:/tmp/.X11-unix -v ${GIT_DIR}:/root/git -v ${CATKIN_DIR}:/root/catkin_ws/ $IMAGE_TAG

elif [[ "$IMAGE_ID" != "" ]]; then
    docker run -it --rm --net=host --runtime nvidia -e DISPLAY=$DISPLAY -v /tmp/.X11-unix/:/tmp/.X11-unix -v ${GIT_DIR}:/root/git -v ${CATKIN_DIR}:/root/catkin_ws/ $IMAGE_ID
fi
