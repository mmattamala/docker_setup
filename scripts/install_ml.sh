#!/bin/bash
set -e

echo "Installing ML packages"

# PyTorch
if [[ "$CUDA" == "11.6.0" ]]; then
    pip3 install --no-cache-dir \
        torch \
        torchvision \
        torchaudio \
        --extra-index-url https://download.pytorch.org/whl/cu116

elif [[ "$CUDA" == "11.7.0" ]]; then
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
