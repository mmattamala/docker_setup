#!/bin/bash
# This file builds docker images
# author: Matias Mattamala
set -e

# Include helpers
source bin/helpers.sh

# Define usage
__usage="
Usage: $(basename $0) --target=TARGET [OPTIONS]

Options:
  --target=<target>    Target to be built: [$(list_targets)]
  --stage=<stage>      Specific stage to be built: [$(list_stages)]
  --no-push            DO NOT push images to DockerHub
  --no-build           DO NOT build images to DockerHub
"

# Default target
TARGET=none
STAGE=all
BUILD_IMAGES="true"
PUSH_IMAGES="true"

# Read arguments
for i in "$@"
do
    case $i in
        --target=*)
            TARGET=${i#*=}
            shift
            ;;
        --stage=*)
            STAGE=${i#*=}
            shift
            ;;
        --no-push)
            PUSH_IMAGES="false"
            shift
            ;;
        --no-build)
            BUILD_IMAGES="false"
            shift
            ;;
        *)
            echo "$__usage"
	        exit 0
            ;;
    esac
done

# Print summary of options
echo " == build.sh =="
echo " Target:       '$TARGET'"
echo " Stage:        '$STAGE'"
echo " Push images:  '$PUSH_IMAGES'"
echo " Build images: '$BUILD_IMAGES'"
echo " =============="

if [[ "$TARGET" == "none" ]]; then
    echo "$__usage"
	exit 0
fi

# Handle different target cases
source targets/$TARGET.sh
check_target_exists $TARGET

# Check stage exists
check_stage_exists $STAGE

# Check if base image exists, otherwise pull it
if [[ "$(docker images -q $BASE_IMAGE 2> /dev/null)" == "" ]]; then
    echo_warning "Base image [$BASE_IMAGE] not found. Pulling from DockerHub..."
    docker pull $BASE_IMAGE
    echo "Done"
fi

# Get architecture of base image
BASE_ARCH=$(docker inspect --format '{{ .Os }}/{{ .Architecture }}' $BASE_IMAGE)

# Prepare stages
LAST_STAGE="$BASE_IMAGE"

if [[ $STAGE == "all" ]]; then
    BUILD_STAGES=$(list_stages)
else
    BUILD_STAGES=$STAGE
    PREVIOUS_STAGE=$(find_previous_stage $STAGE)

    if [[ "$PREVIOUS_STAGE" != "" ]]; then
        LAST_STAGE=${IMAGE_TAG}-${PREVIOUS_STAGE##*-}
    fi
fi

# Build
for BUILD in ${BUILD_STAGES[@]}; do
    NEW_STAGE=${IMAGE_TAG}-${BUILD##*-}

    if [[ "$BUILD_IMAGES" == "true" ]]; then
        echo "Building [$NEW_STAGE] from [$LAST_STAGE]"
        docker buildx build --build-arg BASE_IMAGE=$LAST_STAGE \
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

    fi

    # Push images if requested
    if [[ "$PUSH_IMAGES" == "true" ]]; then
        echo "Pushing [$NEW_STAGE]"
        docker push ${NEW_STAGE}
    fi

    # Update last build
    LAST_STAGE=$NEW_STAGE
done

# If building all the stages, create final tag
if [[ "$STAGE" == "all" ]]; then
    # Make final tag for last image
    docker tag "${LAST_STAGE}" "${IMAGE_TAG}"

    # Push if requested
    if [[ "$PUSH_IMAGES" == "true" ]]; then
        docker push ${IMAGE_TAG}
    fi
fi

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
