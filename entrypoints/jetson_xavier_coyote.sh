#!/bin/bash
# This is to be used in Coyote

# Setup ROS_IP to LPC
ros_network 192.168.0.41

# Install wvn
install_wvn="pip3 install -e /root/git/wild_visual_navigation"

echo "Installing wvn: ${install_wvn}..."
$install_wvn > /dev/null
echo "Done!"