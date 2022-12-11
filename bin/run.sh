#!/bin/bash
# This script launches a container for the target platform
# author: Matias Mattamala
set -e

# Include helpers
source bin/helpers.sh

# Define usage
__usage="
Usage: $(basename $0) --target=TARGET|--image=IMAGE [OPTIONS]

Options:
  -t, --target=<target>        Selected target (available ones): [$(list_targets)]
  -i, --image=<image>          Tag or image id (if not using specific target)
  -g, --git=<git_folder>       Git folder to be mounted (Default: $HOME/git)
  -c, --catkin=<ws_folder>     Catkin workspace folder to be mounted (Default: $HOME/catkin_ws)
  -e, --entrypoint=<file>      Custom script to be executed when running the container
"

# Default target
TARGET=none
IMAGE_ID=none
STAGE=""
GIT_DIR="$HOME/git"
CATKIN_DIR="$HOME/catkin_ws"
ENTRYPOINT_FILE="dummy.sh"

# Read arguments
for i in "$@"; do
    case $i in
        -t=*|--target=*)
            TARGET=${i#*=}
            shift
            ;;
        -s=*|--stage=*)
            STAGE=${i#*=}
            shift
            ;;
        -i=*|--image=*|--image-id=*)
            IMAGE_ID=${i#*=}
            shift
            ;;
        -g=*|--git=*|--git-dir=*)
            GIT_DIR=${i#*=}
            shift
            ;;
        -c=*|--catkin=*|--catkin-dir=*)
            CATKIN_DIR=${i#*=}
            shift
            ;;
        -e=*|--entrypoint=*)
            ENTRYPOINT_FILE=${i#*=}
            shift
            ;;
        *)
            echo "$__usage"
            exit 0
            ;;
    esac
done

ENTRYPOINT_FILEPATH="$(pwd)/entrypoints/${ENTRYPOINT_FILE}"
if [[ "$(check_file_exists $ENTRYPOINT_FILE)" == "false" ]]; then
    ENTRYPOINT_FILEPATH="$(pwd)/entrypoints/dummy.sh"
fi

# Print summary of options
echo " == run.sh =="
echo " Target:          '$TARGET'"
echo " Stage:           '$STAGE'"
echo " Images:          '$IMAGE_ID'"
echo " Mount git:       '$GIT_DIR'"
echo " Mount catkin:    '$CATKIN_DIR'"
echo " Entrypoint file: '$ENTRYPOINT_FILEPATH'"
echo " ============"

if [[ "$TARGET" == "none" && "$IMAGE_ID" == "none" ]]; then
    echo "$__usage"
	exit 0
fi

if [[ "$STAGE" != "" ]]; then
    # Remove number from stage
    STAGE=${STAGE##*-}
fi

# Handle different target cases
if [[ "$TARGET" != "none" ]]; then
    source targets/$TARGET.sh
    check_target_exists

    # Check stage if requested
    if [[ "$STAGE" != "" && "$STAGE" != "all" ]]; then
        IMAGE_TAG=${IMAGE_TAG}-${STAGE##*-}
    fi

else
    IMAGE_TAG=$IMAGE_ID
fi

# Check if image exists, otherwise pull it
if [[ "$(docker images -q $IMAGE_TAG 2> /dev/null)" == "" ]]; then
    echo_warning "Image [$IMAGE_TAG] not found. Pulling from DockerHub..."
    docker pull $IMAGE_TAG
    echo "Done"
fi

# Get architecture of base image and host system
IMAGE_OS=$(docker inspect --format '{{ .Os }})' $IMAGE_TAG)
HOST_OS=$(docker info --format '{{ .OSType }})')

IMAGE_ARCH=$(docker inspect --format '{{ .Architecture }}' $IMAGE_TAG)
HOST_ARCH=$(docker info --format '{{ .Architecture }}')

# Build with emulator if needed
EMULATOR_FLAGS=""
if [[ $(compare_architectures $IMAGE_ARCH $HOST_ARCH) == "false" ]]; then
    echo_warning "Architectures [$IMAGE_ARCH] [$HOST_ARCH] are not the same, running emulation..."
    run_docker_qemu
    EMULATOR_FLAGS="--security-opt seccomp=unconfined"
fi

# Change the gpu flags depending on the platform - Jetson requires different ones
GPU_FLAGS="--runtime nvidia"
if [[ "$JETPACK_VERSION" != "" ]]; then
    GPU_FLAGS="--gpus all"
fi

# Run docker
docker run -it --rm --net=host \
                    $GPU_FLAGS \
                    -e DISPLAY=$DISPLAY \
                    -v /tmp/.X11-unix/:/tmp/.X11-unix \
                    -v ${GIT_DIR}:/root/git \
                    -v ${CATKIN_DIR}:/root/catkin_ws \
                    -v "${ENTRYPOINT_FILEPATH}":/custom_entrypoint.sh \
                    --pull "missing" \
                    $EMULATOR_FLAGS \
                    $IMAGE_TAG
