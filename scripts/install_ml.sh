#!/bin/bash
set -e

echo "Installing ML packages"

# PyTorch
pip3 install --no-cache-dir \
      torch \
      torchvision \
      torchaudio

# Other libraries
pip3 install --no-cache-dir \
      pycuda \
      cupy \
      scikit-learn \
      scikit-image \
      pandas \
      scipy \
      onnx \
      numba \
