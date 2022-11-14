#!/bin/bash
# Installation of other libraries
# author: Matias Mattamala
set -e
echo "Installing ML packages..."
echo "  WITH_CUDA:       $WITH_CUDA"
echo "  CUDA_VERSION:    $CUDA_VERSION"
echo "  CUDA_ARCH_BIN:   $CUDA_ARCH_BIN"
echo "  JETPACK_VERSION: $JETPACK_VERSION"
echo "  ROS_VERSION:     $ROS_VERSION"

# Other basic libraries
pip3 install --no-cache-dir \
      ruamel.yaml \
      shapely==1.7.1 \
      chainer \

