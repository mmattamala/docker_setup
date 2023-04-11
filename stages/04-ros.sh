#!/bin/bash
# Installing ROS noetic
# author: Matias Mattamala
set -e
echo "Installing OpeROS..."
echo "  WITH_CUDA:       $WITH_CUDA"
echo "  CUDA_VERSION:    $CUDA_VERSION"
echo "  CUDA_ARCH_BIN:   $CUDA_ARCH_BIN"
echo "  JETPACK_VERSION: $JETPACK_VERSION"
echo "  ROS_VERSION:     $ROS_VERSION"

# Read arguments
for i in "$@"
do
case $i in
    -t=*|--version=*)
      ROS_VERSION=${i#*=}
      shift
      ;;
esac
done

echo "Installing ROS ${ROS_VERSION} packages"

# Setup sources.list
sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

# Setup keys
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -

# Update repos
apt update -y

# Install ROS
apt install -y ros-${ROS_VERSION}-desktop-full

# Install other packages
apt install -y \
    python3-rosdep \
    python3-rosinstall \
    python3-rosinstall-generator \
    python3-wstool \
    python3-catkin-tools \
    python3-osrf-pycommon

# Rosdep
rm -f "/etc/ros/rosdep/sources.list.d/20-default.list"
rosdep init
rosdep update

# Remove apt repos
rm -rf /var/lib/apt/lists/*
apt-get clean

# Add to .bashrc
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc