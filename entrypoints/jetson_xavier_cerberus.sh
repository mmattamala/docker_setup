#!/bin/bash

# Fix for skimage
export LD_PRELOAD=/usr/local/lib/python3.8/dist-packages/skimage/_shared/../../scikit_image.libs/libgomp-d22c30c5.so.1.0.0

# User specific enviornment configuration
export ENV_WORKSTATION_NAME=jetson
export PROCMAN_DEPUTY=anymal_cerberus_xavier

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

