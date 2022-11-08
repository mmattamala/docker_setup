#!/bin/bash

# Target name
TARGET_NAME="cpu"

# Used to push the images
USERNAME="mmattamala"

# Dockerfile used to build the image
DOCKERFILE="Dockerfile"

# Ubuntu version for the base system
UBUNTU_VERSION="ubuntu20.04"

# ROS version to be installed
ROS_VERSION="noetic"

# CUDA stuff (not required for this target)
WITH_CUDA="false"
CUDA_VERSION=""
CUDA_ARCH_BIN=""

# These variables go in the end since they rely on the previous
# Base image for docker
BASE_IMAGE="ubuntu:${UBUNTU_VERSION}"

# Tag for the created image
IMAGE_TAG="$USERNAME/devel-$TARGET_NAME:$UBUNTU_VERSION-$ROS_VERSION"
