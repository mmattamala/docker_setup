#!/bin/bash

# Force color prompt
export force_color_prompt=yes

# Source .bash_aliases
source /root/.bash_aliases

# Sourcing catkin
echo "source /root/catkin_ws/devel/setup.bash" >> /root/.bashrc

# Source custom entrypoint
source /custom_entrypoint.sh

exec "$@"
