#!/bin/bash
# This defines some bash helpers and aliases

# Prompt colors
# This will output the propmpt like
#   (docker) user@hostname:
#  with:
#    (docker)        in light blue
#    user@hostname   in pink
#    
# For more options, you can design your own: https://bashrcgenerator.com/
#

PS_PREFIX_COLOR="\033[38;5;33m"
PS_USER_COLOR="\033[38;5;204m"
PS_HOST_COLOR="\033[38;5;204m"
PS_DIR_COLOR="\033[38;5;33m"

PS_PREFIX_TEXT="(docker)"

PS_PREFIX="\[$(tput bold)\]\[${PS_PREFIX_COLOR}\]${PS_PREFIX_TEXT}\[$(tput sgr0)\]"
PS_USER="\[$(tput bold)\]\[${PS_USER_COLOR}\]\u\[$(tput sgr0)\]"
PS_HOST="\[$(tput bold)\]\[${PS_HOST_COLOR}\]@\h\[$(tput sgr0)\]"
PS_DIR="\[$(tput bold)\]\[${PS_DIR_COLOR}\]\w\[$(tput sgr0)\]"

PS1="${PS_PREFIX} ${PS_USER}${PS_HOST}:${PS_DIR}\\$ "

ros_network() {
    # This script configures the ROS environment variables according to the route
    # to the ROS_MASTER. ROS_MASTER can either be defined as an evironment variable
    # itself or given as first argument to this script. The ROS_IP and ROS_HOSTNAME
    # are set according to the IP that is sitting on the route to this master. 
    # The ROS_MASTER_URI is also set, using port 11311. ROS_MASTER needs to be defined
    # as a numeric IP address, not a hostname.
    # author: Michal Staniaszek
    
    if [ "$1" ]; then
	    ROS_MASTER="$1"
    fi

    if [ -z "$ROS_MASTER" ]; then
	    ROS_MASTER=127.0.0.1
    fi

    echo "ROS_MASTER:     $ROS_MASTER"

    export ROS_IP=`ip route get $ROS_MASTER | grep "src" | sed 's/.*src \([0-9\.]*\).*/\1/'`
    export ROS_HOSTNAME=$ROS_IP
    export ROS_MASTER_URI="http://$ROS_MASTER:11311/"

    echo "ROS_IP:         $ROS_IP"
    echo "ROS_HOSTNAME:   $ROS_HOSTNAME"
    echo "ROS_MASTER_URI: $ROS_MASTER_URI"
}