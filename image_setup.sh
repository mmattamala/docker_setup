#!/bin/bash

# Host dirs
GIT_WS=$HOME/git
CATKIN_WS=$HOME/docker_ws

USERNAME="mmattamala"
DOCKERFILE="Dockerfile"

UBUNTU_VERSION="ubuntu20.04"
ROS_VERSION="noetic"
WITH_CUDA="true"
CUDA_VERSION="11.8.0"
CUDA_ARCH_BIN="6.1" # For Quadro P2000, check https://developer.nvidia.com/cuda-gpus

GPU_IMAGE_TAG="$USERNAME/devel-gpu:$UBUNTU_VERSION-$ROS_VERSION-cuda$CUDA_VERSION"
CPU_IMAGE_TAG="$USERNAME/devel-cpu:$UBUNTU_VERSION-$ROS_VERSION"

if [[ "$WITH_CUDA" == "true" ]]; then
    BASE_IMAGE="nvidia/cuda:$CUDA_VERSION-cudnn8-devel-$UBUNTU_VERSION"
    IMAGE_TAG=$GPU_IMAGE_TAG
else
    BASE_IMAGE="ubuntu:${UBUNTU_VERSION}"
    IMAGE_TAG=$CPU_IMAGE_TAG
fi
