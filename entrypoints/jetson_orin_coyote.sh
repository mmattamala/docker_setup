#!/bin/bash
# This is to be used in Coyote

# Fix for skimage
export LD_PRELOAD=/usr/local/lib/python3.8/dist-packages/skimage/_shared/../../scikit_image.libs/libgomp-d22c30c5.so.1.0.0

# Setup ROS_IP to LPC
echo "ros_network 192.168.0.41 > /dev/null" >> ~/.bashrc

# Install wvn
install_wvn="pip3 install -e /root/git/wild_visual_navigation"

echo "Installing wvn: ${install_wvn}..."
$install_wvn > /dev/null
echo "Done!"

# Export environment name
export ENV_WORKSTATION_NAME=jetson

# Procman
source ~/catkin_ws/devel/setup.bash && rosrun procman_ros deputy -i orin
