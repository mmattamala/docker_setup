#!/bin/bash
# Installation of ML libraries (Pytorch)
# author: Matias Mattamala
set -e
echo "Installing ML packages..."
echo "  WITH_CUDA:       $WITH_CUDA"
echo "  CUDA_VERSION:    $CUDA_VERSION"
echo "  CUDA_ARCH_BIN:   $CUDA_ARCH_BIN"
echo "  JETPACK_VERSION: $JETPACK_VERSION"
echo "  ROS_VERSION:     $ROS_VERSION"

# Basic numerical and ML libraries
pip3 install --no-cache-dir \
      numpy \
      seaborn \
      scikit-learn \
      scikit-image \
      pandas \
      scipy==1.8.1 \
      onnx \
      numba \


# PyTorch
echo "Installing Pytorch for CUDA [$CUDA_VERSION]"
if [[ "$CUDA_VERSION" == "10.2.0" ]]; then
    # Manual compilation for Jetson
    # Instructions here: https://forums.developer.nvidia.com/t/pytorch-for-jetson/72048

    if [[ "$JETPACK_VERSION" != "" ]]; then
        echo "Installing Pytorch 10.1.2 due to cudnn8"
        # 
        pip3 install ninja
        pip3 install scikit-build

        apt install g++-7 -y
        ln -sf /usr/bin/gcc-7 /usr/local/cuda/bin/gcc
        ln -sf /usr/bin/g++-7 /usr/local/cuda/bin/g++

        export USE_NCCL=0
        export USE_DISTRIBUTED=0
        export USE_QNNPACK=0
        export USE_PYTORCH_QNNPACK=0
        export TORCH_CUDA_ARCH_LIST="$CUDA_ARCH_BIN"

        export PYTORCH_BUILD_VERSION=1.10.2  # without the leading 'v'
        export PYTORCH_BUILD_NUMBER=1

        # Clone Pytorch
        cd /
        git clone --recursive --branch v$PYTORCH_BUILD_VERSION https://github.com/pytorch/pytorch --depth 1
        cd pytorch
        python3 setup.py bdist_wheel

    else
        # Normal installation
        echo "Warning: Latest Pytorch available for CUDA 10.2.0 is torch 1.12.1"
        pip3 install --no-cache-dir \
            torch==1.12.1+cu102 \
            torchvision==0.13.1+cu102 \
            torchaudio==0.12.1 \
            --extra-index-url https://download.pytorch.org/whl/cu102
    fi

elif [[ "$CUDA_VERSION" == "11.4.0" ]]; then
    # Manual compilation for Jetson
    if [[ "$JETPACK_VERSION" != "" ]]; then
        echo "TODO - install pytorch"

    else
        # Normal installation
        echo "Warning: PyTorch not available for CUDA 11.4. Installing for CUDA 11.3 instead"
        echo "Warning: Latest Pytorch available for CUDA 10.3.0 is torch 1.12.1"
        pip3 install --no-cache-dir \
            torch==1.12.1+cu113 \
            torchvision==0.13.1+cu113 \
            torchaudio==0.12.1 \
            --extra-index-url https://download.pytorch.org/whl/cu113
    fi

elif [[ "$CUDA_VERSION" == "11.6.0" ]]; then
    pip3 install --no-cache-dir \
        torch \
        torchvision \
        torchaudio \
        --extra-index-url https://download.pytorch.org/whl/cu116

elif [[ "$CUDA_VERSION" == "11.7.0" ]]; then
    pip3 install --no-cache-dir \
        torch \
        torchvision \
        torchaudio \
        --extra-index-url https://download.pytorch.org/whl/cu117

else
    pip3 install --no-cache-dir \
        torch \
        torchvision \
        torchaudio
fi


# # Pycuda
# pip3 install --no-cache-dir pycuda

# CuPy
if [[ "$CUDA_VERSION" == "10.2.0" ]]; then
    if [[ "$JETPACK_VERSION" != "" ]]; then
        pip3 install --no-cache-dir cupy-cuda102 -f https://pip.cupy.dev/aarch64
    else
        pip3 install --no-cache-dir cupy-cuda102
    fi

elif [[ "$CUDA_VERSION" == "11.4.0" ]]; then
    pip3 install --no-cache-dir cupy-cuda114
    python3 -m cupyx.tools.install_library --cuda 11.4 --library cutensor

elif [[ "$CUDA_VERSION" == "11.6.0" ]]; then
    pip3 install --no-cache-dir cupy-cuda116
    python3 -m cupyx.tools.install_library --cuda 11.6 --library cutensor

elif [[ "$CUDA_VERSION" == "11.7.0" ]]; then
    pip33 install --no-cache-dir cupy-cuda117
    python3 -m cupyx.tools.install_library --cuda 11.7 --library cutensor

else
    pip33 install --no-cache-dir cupy
fi

