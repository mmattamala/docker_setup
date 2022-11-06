#!/bin/bash
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
if [[ "$TARGET" == "jetson" ]]; then
    echo "Building images for target [$TARGET]"
    echo "Skipping"

elif [[ "$TARGET" == "desktop" ]]; then
    echo "Building images for target [$TARGET]"

		# Build docker
    echo "Building dockerfile..."
    sudo docker build --build-arg BASE_IMAGE=$BASE_IMAGE \
                      --build-arg USERNAME=$USERNAME \
                      --build-arg UBUNTU_VERSION=$UBUNTU_VERSION \
                      --build-arg ROS_VERSION=$ROS_VERSION \
                      --build-arg WITH_CUDA=$WITH_CUDA \
                      --build-arg CUDA_VERSION=$CUDA_VERSION \
                      --build-arg CUDA_ARCH_BIN=$CUDA_ARCH_BIN \
                      --network=host \
                      -t $IMAGE_TAG -f $DOCKERFILE .

else
    echo "Error: unsupported target [$TARGET]"
fi