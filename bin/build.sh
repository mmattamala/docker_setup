#!/bin/bash
# This file builds docker images given a target

set -e

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
source targets/$TARGET.sh

# Build image
sudo docker build --build-arg BASE_IMAGE=$BASE_IMAGE \
                  --build-arg UBUNTU_VERSION=$UBUNTU_VERSION \
                  --build-arg ROS_VERSION=$ROS_VERSION \
                  --build-arg WITH_CUDA=$WITH_CUDA \
                  --build-arg CUDA_VERSION=$CUDA_VERSION \
                  --build-arg CUDA_ARCH_BIN=$CUDA_ARCH_BIN \
                  --network=host \
                  -t $IMAGE_TAG -f $DOCKERFILE .



# # Handle different target cases
# if [[ "$TARGET" == "jetson" ]]; then
#     echo "Building images for target [$TARGET]"
#     echo "Skipping"

# elif [[ "$TARGET" == "cpu" || "$TARGET" == "gpu" ]]; then
#     echo "Building images for target [$TARGET]"

# 		# Build docker
#     echo "Building dockerfile..."


# else
#     echo "Error: unsupported target [$TARGET]. "
# fi