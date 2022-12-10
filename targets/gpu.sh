#!/bin/bash
# author: Matias Mattamala

# Target name
TARGET_NAME="gpu"

# Used to push the images
USERNAME="mmattamala"

# Dockerfile used to build the image
DOCKERFILE="Dockerfile"

# Ubuntu version for the base system
UBUNTU_VERSION="ubuntu20.04"

# ROS version to be installed
ROS_VERSION="noetic"

# CUDA stuff
# For CUDA_ARCH_BIN:
# check here:
#    https://developer.nvidia.com/cuda-gpus
# or better run (David Wisth's advice):
#    nvidia-smi --query-gpu=compute_cap --format=csv
WITH_CUDA="true"
CUDA_VERSION="11.6.0"
CUDA_ARCH_BIN="'5.2;6.1;7.5;8.6'" # for DRS laptops
JETPACK_VERSION=""

# These variables go in the end since they rely on the previous ones
# Base image for docker
BASE_IMAGE="nvidia/cuda:$CUDA_VERSION-cudnn8-devel-$UBUNTU_VERSION"

# Tag for the created image
IMAGE_TAG="$USERNAME/devel-$TARGET_NAME:$UBUNTU_VERSION-$ROS_VERSION-cuda$CUDA_VERSION"
