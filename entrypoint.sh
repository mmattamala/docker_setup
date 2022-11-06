#!/bin/bash
set -e

# Sourcing catkin
echo "source /root/catkin_ws/devel/setup.bash" >> /root/.bashrc

exec "$@"
