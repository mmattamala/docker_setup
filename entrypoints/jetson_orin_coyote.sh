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

# User specific enviornment configuration
export ENV_WORKSTATION_NAME=jetson
export PROCMAN_DEPUTY=anymal_coyote_orin

# Assess if catkin_ws can be sourced
catkin_ws=$(echo ~/catkin_ws/devel/setup.bash)
if [ -f "$catkin_ws" ]
then
    # Assess if procman is build
    out=$(source ~/.bashrc && source ~/catkin_ws/devel/setup.bash && echo roscd procman_ros)
    ref=$(echo roscd: No such package/stack \'procman_ros\')
    if [ $out == $ref ]; then
        source ~/.bashrc && source ~/catkin_ws/devel/setup.bash  
        echo "Warning: procman_ros is not build within the catkin_ws. Therefore the deputy cannot be started!"
    else
        source ~/.bashrc && source ~/catkin_ws/devel/setup.bash  && rosrun procman_ros deputy -i $PROCMAN_DEPUTY
    fi
else
    echo "Warning: catkin_ws does not exist!"
fi
