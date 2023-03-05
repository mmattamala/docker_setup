#!/bin/bash
# Installation of CUDA for Jetson devices
# author: Matias Mattamala
set -e
echo "Installing CUDA packages..."
echo "  WITH_CUDA:       $WITH_CUDA"
echo "  CUDA_VERSION:    $CUDA_VERSION"
echo "  CUDA_ARCH_BIN:   $CUDA_ARCH_BIN"
echo "  JETPACK_VERSION: $JETPACK_VERSION"
echo "  ROS_VERSION:     $ROS_VERSION"

if [[ "$WITH_CUDA" != "" ]]; then
    if [[ "$JETPACK_VERSION" != "" ]]; then
        if [[ "$JETPACK_VERSION" == "r32.5.0" ]]; then
            # Get public key
            apt-key adv --fetch-key http://repo.download.nvidia.com/jetson/jetson-ota-public.asc

            # Manually add apt
            touch /etc/apt/sources.list.d/nvidia-l4t-apt-source.list
            echo "deb https://repo.download.nvidia.com/jetson/common r32.5 main" > /etc/apt/sources.list.d/nvidia-l4t-apt-source.list
            echo "deb https://repo.download.nvidia.com/jetson/t194 r32.5 main" >> /etc/apt/sources.list.d/nvidia-l4t-apt-source.list

        elif [[ "$JETPACK_VERSION" == "r34.1.1" ]]; then
            # Get public key
            apt-key adv --fetch-key http://repo.download.nvidia.com/jetson/jetson-ota-public.asc

            # Manually add apt
            touch /etc/apt/sources.list.d/nvidia-l4t-apt-source.list
            echo "deb https://repo.download.nvidia.com/jetson/common r34.1 main" > /etc/apt/sources.list.d/nvidia-l4t-apt-source.list
            echo "deb https://repo.download.nvidia.com/jetson/t194 r34.1 main" >> /etc/apt/sources.list.d/nvidia-l4t-apt-source.list    

        elif [[ "$JETPACK_VERSION" == "r35.1.0" ]]; then
            # Get public key
            apt-key adv --fetch-key http://repo.download.nvidia.com/jetson/jetson-ota-public.asc

            # Manually add apt
            touch /etc/apt/sources.list.d/nvidia-l4t-apt-source.list
            echo "deb https://repo.download.nvidia.com/jetson/common r35.1 main" > /etc/apt/sources.list.d/nvidia-l4t-apt-source.list
            echo "deb https://repo.download.nvidia.com/jetson/t194 r35.1 main" >> /etc/apt/sources.list.d/nvidia-l4t-apt-source.list
        else
            echo "Not supported Jetpack version [$JETPACK_VERSION]"
            exit 1
        fi

        # Update apt
        apt update

        # Install CUDA
        apt install -y cuda-toolkit-* 

        # Install CUDNN
        apt install -y libcudnn*-dev
        # # Install VPI
        apt install -y vpi*-dev
        #                vpi2-samples \
        #                python3.8-vpi2
        # # Install TensorRT
        apt install -y tensorrt
        #                python3-libnvfer \
        # 	             python3-libnvinfer-dev

        # Install jetson stats
        pip3 install -U jetson-stats

    # else
    #     # Desktop installation
    #     DISTRO="$(echo $UBUNTU_VERSION | tr -d .)"
    #     ARCH="$(uname -m)"
    #     wget https://developer.download.nvidia.com/compute/cuda/repos/$DISTRO/$ARCH/cuda-keyring_1.0-1_all.deb
    #     sudo dpkg -i cuda-keyring_1.0-1_all.deb
    fi
    
    # Export paths
    echo "export PATH=/usr/local/cuda/bin:$PATH" >> /root/.bashrc
    echo "export CPATH=/usr/local/cuda/include:$CPATH" >> /root/.bashrc
    echo "export LD_LIBRARY_PATH=/usr/local/cuda/lib64:$LD_LIBRARY_PATH" >> /root/.bashrc
    echo "export CUDA_HOME=/usr/local/cuda" >> /root/.bashrc
    echo "export CUDA_PATH=/usr/local/cuda" >> /root/.bashrc
    echo "export TORCH_CUDA_ARG_LIST=$CUDA_ARCH_BIN" >> /root/.bashrc
    echo "export CUDA_ARCH_BIN=$CUDA_ARCH_BIN" >> /root/.bashrc
    source /root/.bashrc

    # Remove apt repos
    rm -rf /var/lib/apt/lists/*
    apt-get clean
fi
