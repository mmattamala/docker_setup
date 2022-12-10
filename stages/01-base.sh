#!/bin/bash
# Installation of default packages for all the targets
# author: Matias Mattamala
set -e
echo "Installing base packages..."

# Update first
apt update

# Install base packages
apt install -y \
    sudo \
    lsb-release \
    ca-certificates \
    apt-utils \
    gnupg2 \
    locate \
    curl \
    wget \
    git \
    vim \
    nano \
    gedit \
    tmux \
    unzip \
    iputils-ping \
    iproute2 \
    net-tools \
    htop \
    iotop \
    iftop \
    nmap \
    software-properties-common \
    build-essential \
    ninja-build \
    gdb \
    pkg-config \
    cmake \
    zsh \
    tzdata \
    clang-format \
    clang-tidy \
    xterm \
    gnome-terminal \
    dialog \
    tasksel \
    meld \
    figlet \
    wmctrl \
    tmux \
    usbutils \
    libtbb2 \
    libtbb-dev \
    libjpeg-dev \
    libpng-dev \
    libtiff-dev \
    libavformat-dev \
    libpq-dev \
    libxine2-dev \
    libglew-dev \
    libtiff5-dev \
    zlib1g-dev \
    libjpeg-dev \
    libavcodec-dev \
    libavformat-dev \
    libavutil-dev \
    libpostproc-dev \
    libswscale-dev \
    libeigen3-dev \
    libgtk2.0-dev \
    libopenblas-dev \
    libopenblas-base \
    liblapack-dev \
    liblapacke-dev \
    libopenmpi-dev \
    openmpi-bin \
    openmpi-common \
    gfortran \
    libomp-dev \
    python3 \
    python3-dev \
    python3-pip \
    python-dev \
    python-numpy \
    python3-dev \
    python3-numpy \
    python3-matplotlib \
    python3-vtk7

# # Python
# curl https://bootstrap.pypa.io/get-pip.py -o get-pip.py
# python3 get-pip.py

# pip install --no-cache-dir \
#                 numpy \
#                 matplotlib
# apt install -y python3-vtk7


# Remove apt repos
rm -rf /var/lib/apt/lists/*
apt-get clean

# Patch LAPACK
sudo ln -sf /usr/include/lapacke.h /usr/include/$(uname -m)-linux-gnu
sudo ln -sf /usr/include/lapack.h /usr/include/$(uname -m)-linux-gnu

# Enable mouse for tmux
echo """# Make mouse useful in copy mode \
      setw -g mouse on """ >> ~/.tmux.conf
