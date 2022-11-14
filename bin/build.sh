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
  -p, --push                   Push images to DockerHub
"

# Default target
TARGET=none
STAGE=all
PUSH_IMAGES="false"

# Read arguments
for i in "$@"
do
    case $i in
        -t=*|--target=*)
            TARGET=${i#*=}
            echo "[build.sh]: Building target: '$TARGET'"
            shift
            ;;
        -s=*|--stage=*)
            STAGE=${i#*=}
            echo "[build.sh]: Selected stages: '$STAGE'"
            shift
            ;;
        -p|--push)
            PUSH_IMAGES="true"
            echo "[build.sh]: Push images after each build: '$PUSH_IMAGES'"
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

    echo "$PREVIOUS_STAGE"

    if [[ "$PREVIOUS_STAGE" != "" ]]; then
        LAST_STAGE=${IMAGE_TAG}-${PREVIOUS_STAGE##*-}
    else
        LAST_STAGE=${IMAGE_TAG}
    fi
    
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
    
    # Push images if requested
    if [[ "$PUSH_IMAGES" == "true" ]]; then
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