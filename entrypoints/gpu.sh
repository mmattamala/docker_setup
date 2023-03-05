#!/bin/bash
# This is to be used on my local machine

# Install wvn
install_wvn="pip3 install -e /root/git/wild_visual_navigation"

echo "Installing wvn: ${install_wvn}..."
$install_wvn > /dev/null
echo "Done!"

source /root/.bashrc