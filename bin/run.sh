#!/bin/bash
# This script launches a container for the target platform
set -e

# Include helpers
source bin/helpers.sh

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
    -i=*|--image=*|--image-id=*)
        IMAGE_ID=${i#*=}
        echo "[run.sh]: User-set IMAGE_ID is: '$IMAGE_ID'"
        shift
        ;;
    -g=*|--git=*|--git-dir=*)
        GIT_DIR=${i#*=}
        echo "[run.sh]: User-set GIT_DIR is: '$GIT_DIR'"
        shift
        ;;
    -c=*|--catkin=*|--catkin-dir=*)
        CATKIN_DIR=${i#*=}
        echo "[run.sh]: User-set CATKIN_DIR is: '$CATKIN_DIR'"
        shift
        ;;
esac
done


# Handle different target cases
if [[ "$TARGET" != "none" ]]; then
    source targets/$TARGET.sh
    check_target_exists
else
    BASE_IMAGE=$IMAGE_ID
fi

# Get architecture of base image and host system
BASE_ARCH=$(docker inspect --format '{{ .Os }}/{{ .Architecture }}' $BASE_IMAGE)
HOST_ARCH=$(docker info --format '{{ .OSType }}/{{ .Architecture }}')

# Build with emulator if needed
EMULATOR_FLAGS=""
if [[ "$BASE_ARCH" != "$HOST_ARCH" ]]; then
    run_docker_qemu
    EMULATOR_FLAGS="--security-opt seccomp=unconfined"
fi

# Change the gpu flags depending on the platform - Jetson requires different ones
GPU_FLAGS="--runtime nvidia"
if [[ "$JETPACK_VERSION" != "" ]]; then
    GPU_FLAGS="--gpus all"
fi

# Run docker
if [[ "$TARGET" != "none" ]]; then
    docker run -it --rm --net=host $GPU_FLAGS -e DISPLAY=$DISPLAY -v /tmp/.X11-unix/:/tmp/.X11-unix -v ${GIT_DIR}:/root/git -v ${CATKIN_DIR}:/root/catkin_ws/ $EMULATOR_FLAGS $IMAGE_TAG

elif [[ "$IMAGE_ID" != "" ]]; then
    docker run -it --rm --net=host $GPU_FLAGS -e DISPLAY=$DISPLAY -v /tmp/.X11-unix/:/tmp/.X11-unix -v ${GIT_DIR}:/root/git -v ${CATKIN_DIR}:/root/catkin_ws/ $EMULATOR_FLAGS $IMAGE_ID
fi
