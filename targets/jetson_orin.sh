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
CUDA_VERSION="11.4.0"
CUDA_ARCH_BIN="8.7" # For Orin, check https://developer.nvidia.com/cuda-gpus
JETPACK_VERSION="r35.1.0"

# These variables go in the end since they rely on the previous ones
# Base image for docker
#BASE_IMAGE="arm64v8/ubuntu:focal"
BASE_IMAGE="nvcr.io/nvidia/l4t-base:r35.1.0"

# Tag for the created image
IMAGE_TAG="$USERNAME/devel-$TARGET_NAME:$UBUNTU_VERSION-$ROS_VERSION-cuda$CUDA_VERSION-$JETPACK_VERSION"
