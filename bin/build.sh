#!/bin/bash
# This file builds docker images given a target
set -e

# Include helpers
source bin/helpers.sh

# Define usage
__usage="
Usage: $(basename $0) --target=TARGET [OPTIONS]

Options:
  -t, --target=<target>        Target to be built: [$(list_targets)]
  -s, --stage=<stage>          Last stage to be built: [$(list_stages)]
"

# Default target
TARGET=none
STAGE=all

# Read arguments
for i in "$@"
do
    case $i in
        -t=*|--target=*)
            TARGET=${i#*=}
            echo "[build.sh]: User-set target type is: '$TARGET'"
            shift
            ;;
        -s=*|--stage=*)
            STAGE=${i#*=}
            echo "[build.sh]: User-set stage type is: '$STAGE'"
            shift
            ;;
        *)
            echo "$__usage"
	        exit 0
            ;;
    esac
done

if [[ "$TARGET" == "none" ]]; then
    echo "$__usage"
	exit 0
fi

# Handle different target cases
source targets/$TARGET.sh
check_target_exists $TARGET

# Check stage exists
check_stage_exists $STAGE

# Get architecture of base image
BASE_ARCH=$(docker inspect --format '{{ .Os }}/{{ .Architecture }}' $BASE_IMAGE)

# Prepare stages
if [[ $STAGE == "all" ]]; then
    BUILD_STAGES=$(list_stages)
    LAST_STAGE="$BASE_IMAGE"
else
    BUILD_STAGES=$STAGE
    PREVIOUS_STAGE=$(find_previous_stage $STAGE)
    LAST_STAGE=${IMAGE_TAG}-${PREVIOUS_STAGE##*-}
fi

# Build
for BUILD in ${BUILD_STAGES[@]}; do
    NEW_STAGE=${IMAGE_TAG}-${BUILD##*-}
    echo "Building [$NEW_STAGE] from [$LAST_STAGE]"

    sudo docker buildx build --build-arg BASE_IMAGE=$LAST_STAGE \
                         --build-arg SCRIPT=${BUILD}.sh \
                         --build-arg UBUNTU_VERSION=${UBUNTU_VERSION} \
                         --build-arg ROS_VERSION=${ROS_VERSION} \
                         --build-arg WITH_CUDA=${WITH_CUDA} \
                         --build-arg CUDA_VERSION=${CUDA_VERSION} \
                         --build-arg CUDA_ARCH_BIN=${CUDA_ARCH_BIN} \
                         --build-arg JETPACK_VERSION=${JETPACK_VERSION} \
                         --network=host \
                         --platform ${BASE_ARCH} \
                         -t ${NEW_STAGE} -f $DOCKERFILE .
    
    # Update last build
    LAST_STAGE=$NEW_STAGE
done

# Make final tag for last image
docker tag "${LAST_STAGE}" "${IMAGE_TAG}"


# BUILD_STAGES=(base cuda opencv ros ml)
# # Start building
# sudo docker buildx build --build-arg BASE_IMAGE=$BASE_IMAGE \
#                          --build-arg SCRIPT=${BUILD}.sh \
#                          --build-arg UBUNTU_VERSION=${UBUNTU_VERSION} \
#                          --build-arg ROS_VERSION=${ROS_VERSION} \
#                          --build-arg WITH_CUDA=${WITH_CUDA} \
#                          --build-arg CUDA_VERSION=${CUDA_VERSION} \
#                          --build-arg CUDA_ARCH_BIN=${CUDA_ARCH_BIN} \
#                          --build-arg JETPACK_VERSION=${JETPACK_VERSION} \
#                          --network host \
#                          --platform ${BASE_ARCH} \
#                          --target ${STAGE} \
#                          -t ${IMAGE_TAG} -f $DOCKERFILE .