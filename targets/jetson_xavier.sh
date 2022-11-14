#!/bin/bash
# author: Matias Mattamala

# Target name
TARGET_NAME="jetson"

# Used to push the images
USERNAME="mmattamala"

# Dockerfile used to build the image
DOCKERFILE="Dockerfile"

# Ubuntu version for the base system
UBUNTU_VERSION="ubuntu20.04"

# ROS version to be installed
ROS_VERSION="noetic"

# CUDA stuff
WITH_CUDA="true"
CUDA_VERSION="10.2.0"
CUDA_ARCH_BIN="7.2" # For Xavier, check https://developer.nvidia.com/cuda-gpus
JETPACK_VERSION="r32.5.0"

# These variables go in the end since they rely on the previous ones
# Base image for docker
BASE_IMAGE="arm64v8/ubuntu:focal"

# Tag for the created image
IMAGE_TAG="$USERNAME/devel-$TARGET_NAME:$UBUNTU_VERSION-$ROS_VERSION-cuda$CUDA_VERSION-$JETPACK_VERSION"
