#!/bin/bash
set -e

# Force color prompt
export force_color_prompt=yes

# Sourcing catkin
echo "source /root/catkin_ws/devel/setup.bash" >> /root/.bashrc

exec "$@"
