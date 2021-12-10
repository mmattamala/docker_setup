#!/bin/bash

# Fix for skimage
export LD_PRELOAD=/usr/local/lib/python3.8/dist-packages/skimage/_shared/../../scikit_image.libs/libgomp-d22c30c5.so.1.0.0

# Export environment name
export ENV_WORKSTATION_NAME=jetson

source ~/.bashrc && source ~/catkin_ws/devel/setup.bash  && rosrun procman_ros deputy -i anymal_cerberus_xavier

# Procman
variable=$(/path/to/command)
out=$(source ~/.bashrc && source ~/catkin_ws/devel/setup.bash && echo roscd procman_ros)
echo $out
ref=$(echo roscd: No such package/stack \'procman_ros\')
echo $ref

if [ $out == $ref ]; then
  source ~/.bashrc && source ~/catkin_ws/devel/setup.bash  
else
  source ~/.bashrc && source ~/catkin_ws/devel/setup.bash  && rosrun procman_ros deputy -i anymal_cerberus_xavier
  echo Running Procman
fi
