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

# Source bashrc to get env variables
source /root/.bashrc

install_gcc7()
{
    sudo apt update
    apt install g++-7 -y
    ln -sf /usr/bin/gcc-7 /usr/local/cuda/bin/gcc
    ln -sf /usr/bin/g++-7 /usr/local/cuda/bin/g++ # Maybe useles with the fix below

    # Set the C and C++ compiler explicitly
    export CC=/usr/bin/gcc-7
    export CXX=/usr/bin/g++-7
}

# Helper functions
build_and_install_pytorch()
{
    pip3 install ninja
    pip3 install scikit-build
    pip3 install typing-extensions

    # Flags for Pytorch
    export USE_NCCL=0
    export USE_FBGEMM=0
    export USE_KINETO=0
    export BUILD_TEST=0
    export USE_MKLDNN=0
    export USE_ITT=0
    export BUILD_CAFFE2=0
    export USE_DISTRIBUTED=0
    export USE_QNNPACK=0
    export USE_PYTORCH_QNNPACK=0
    export TORCH_CUDA_ARCH_LIST="$CUDA_ARCH_BIN"

    export PYTORCH_BUILD_VERSION=$1  # without the leading 'v'
    export PYTORCH_BUILD_NUMBER=1

    # Clone custom patched Pytorch
    PYTORCH_FOLDER=/pytorch
    cd /
    git clone --recursive --depth 1 --branch v$PYTORCH_BUILD_VERSION https://github.com/mmattamala/pytorch-jetson $PYTORCH_FOLDER
    cd $PYTORCH_FOLDER

    # Build
    python3 setup.py bdist_wheel

    # Install
    cd /
    pip3 install $PYTORCH_FOLDER/dist/*.whl

    # Remove folder
    cd /
    rm -rf $PYTORCH_FOLDER
}

build_and_install_torchvision()
{
    export BUILD_VERSION=$1  # without the leading 'v'

    # Clone torchvision
    TORCHVISION_FOLDER=/torchvision
    cd /
    git clone --branch v$BUILD_VERSION https://github.com/pytorch/vision $TORCHVISION_FOLDER
    cd $TORCHVISION_FOLDER

    # Install
    python3 setup.py install --user

    # Remove folder
    cd /
    rm -rf $TORCHVISION_FOLDER
}


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
if [[ "$CUDA_VERSION" == "" ]]; then
    echo "Installing PyTorch for CPU"
    pip3 install torch torchvision torchaudio --extra-index-url https://download.pytorch.org/whl/cpu

elif [[ "$CUDA_VERSION" == "10.2.0" ]]; then
    # Manual compilation for Jetson
    # Instructions here: https://forums.developer.nvidia.com/t/pytorch-for-jetson/72048

    if [[ "$JETPACK_VERSION" != "" ]]; then

        # Install PyTorch
        pip3 install --upgrade gdown
        success_pytorch=$(gdown https://drive.google.com/uc?id=1cc1eZgtANkc8IgD3eyfCS0kBWqgigK4d)
        success_torchvision=$(gdown https://drive.google.com/uc?id=16cG8tZKnwNTMqnZ5IUUgk6lHENyLomtI)

        if [[ "$success_pytorch" == "" && "$success_torchvision" == "" ]]; then
            echo "Installing PyTorch from precompiled wheel (Google Drive)"
            pip3 install torch-1.10.2-cp38-cp38-linux_aarch64.whl
            rm torch-1.10.2-cp38-cp38-linux_aarch64.whl

            echo "Installing Torchvision from precompiled wheel (Google Drive)"
            pip3 install torchvision-0.11.0a0+fa347eb-cp38-cp38-linux_aarch64.whl
            rm torchvision-0.11.0a0+fa347eb-cp38-cp38-linux_aarch64.whl

        else
            echo "Compiling PyTorch 1.10.2"
            export PYTORCH_VERSION=1.10.2  # without the leading 'v'
            export TORCHVISION_VERSION=0.11.1

            # Build and install pytorch
            install_gcc7
            build_and_install_pytorch $PYTORCH_VERSION

            # Torchvision
            build_and_install_torchvision $TORCHVISION_VERSION
        fi

    else
        # Normal installation
        echo "Warning: Latest PyTorch available for CUDA 10.2.0 is torch 1.12.1"
        pip3 install --no-cache-dir \
            torch==1.12.1+cu102 \
            torchvision==0.13.1+cu102 \
            torchaudio==0.12.1 \
            --extra-index-url https://download.pytorch.org/whl/cu102
    fi

elif [[ "$CUDA_VERSION" == "11.4.0" ]]; then
    # Manual compilation for Jetson
    if [[ "$JETPACK_VERSION" != "" ]]; then
        echo "Warning: Latest PyTorch available for CUDA 11.4.0 is torch 1.12.0"
        export PYTORCH_VERSION=1.12.0  # without the leading 'v'
        export TORCHVISION_VERSION=0.13.0

        #wget https://developer.download.nvidia.com/compute/redist/jp/v50/pytorch/torch-1.12.0a0+2c916ef.nv22.3-cp38-cp38-linux_aarch64.whl -O torch-1.12.0-cp38-cp38m-linux_aarch64.whl
        #pip3 install Cython
        #pip3 install numpy torch-1.12.0-cp38-cp38m-linux_aarch64.whl
        #rm torch-1.12.0-cp38-cp38m-linux_aarch64.whl
        build_and_install_pytorch $PYTORCH_VERSION

        # torchvision v0.13.0
        build_and_install_torchvision $TORCHVISION_VERSION

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
echo "Installing CuPy for CUDA [$CUDA_VERSION]"

CUPY_REPO=""
if [[ "$JETPACK_VERSION" != "" ]]; then
    echo "CuPy for aarch64..."
    CUPY_REPO="-f https://pip.cupy.dev/aarch64"
fi

# Install with flags
if [[ "$CUDA_VERSION" == "" ]]; then
    echo "Skipping CuPy as CUDA is not available"

elif [[ "$CUDA_VERSION" == "10.2.0" ]]; then
    pip3 install --no-cache-dir cupy-cuda102 $CUPY_REPO

elif [[ "$CUDA_VERSION" == "11.4.0" ]]; then
    pip3 install --no-cache-dir cupy-cuda114 $CUPY_REPO
    # python3 -m cupyx.tools.install_library --cuda 11.4 --library cutensor

elif [[ "$CUDA_VERSION" == "11.6.0" ]]; then
    pip3 install --no-cache-dir cupy-cuda116 $CUPY_REPO
    # python3 -m cupyx.tools.install_library --cuda 11.6 --library cutensor

elif [[ "$CUDA_VERSION" == "11.7.0" ]]; then
    pip3 install --no-cache-dir cupy-cuda117 $CUPY_REPO
    # python3 -m cupyx.tools.install_library --cuda 11.7 --library cutensor

else
    echo "Installing cupy without specify the CUDA version, it will compile it..."
    pip3 install --no-cache-dir cupy
fi

echo "Installing Pytorch Geometric for CUDA [$CUDA_VERSION]"
if [[ "$CUDA_VERSION" == "" ]]; then
    echo "Skipping Torch Geometric as CUDA is not available"

elif [[ "$JETPACK_VERSION" != "" ]]; then
    echo "Building torch_geometric from scratch"
    export LIBRARY_PATH="/usr/local/cuda/lib64:${LIBRARY_PATH}"
    export TORCH_CUDA_ARCH_LIST="$CUDA_ARCH_BIN"
    export FORCE_CUDA=1

    pip3 install -v --no-cache-dir \
            torch-scatter \
            torch-sparse \
            torch-cluster \
            torch-spline-conv \
            torch-geometric
else
    if [[ "$CUDA_VERSION" == "10.2.0" ]]; then
            pip3 install --no-cache-dir \
                              pyg-lib \
                              torch-scatter \
                              torch-sparse \
                              torch-cluster \
                              torch-spline-conv \
                              torch-geometric \
                              -f https://data.pyg.org/whl/torch-1.12.0+cu102.html

    elif [[ "$CUDA_VERSION" == "11.6.0" ]]; then
        pip3 install --no-cache-dir \
                        pyg-lib \
                        torch-scatter \
                        torch-sparse \
                        torch-cluster \
                        torch-spline-conv \
                        torch-geometric \
                        -f https://data.pyg.org/whl/torch-1.13.0+cu116.html

    elif [[ "$CUDA_VERSION" == "11.7.0" ]]; then
        pip3 install --no-cache-dir \
                        pyg-lib \
                        torch-scatter \
                        torch-sparse \
                        torch-cluster \
                        torch-spline-conv \
                        torch-geometric \
                        -f https://data.pyg.org/whl/torch-1.13.0+cu117.html

    else
        export LIBRARY_PATH="/usr/local/cuda/lib64:${LIBRARY_PATH}"
        export TORCH_CUDA_ARCH_LIST="$CUDA_ARCH_BIN"
        export FORCE_CUDA=1
        pip3 install --no-cache-dir \
                        torch-scatter \
                        torch-sparse \
                        torch-cluster \
                        torch-spline-conv \
                        torch-geometric
    fi
fi

