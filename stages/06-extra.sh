#!/bin/bash
# Installation of other libraries
# author: Matias Mattamala
set -e
echo "Installing ML packages..."
echo "  WITH_CUDA:       $WITH_CUDA"
echo "  CUDA_VERSION:    $CUDA_VERSION"
echo "  CUDA_ARCH_BIN:   $CUDA_ARCH_BIN"
echo "  JETPACK_VERSION: $JETPACK_VERSION"
echo "  ROS_VERSION:     $ROS_VERSION"

# Other basic libraries
pip3 install --no-cache-dir \
      ruamel.yaml \
      shapely==1.7.1 \
      chainer \

# Other missing libraries
apt update
apt install -y libsystemd-dev
pip3 install pystemd

# Other libraries for graph neural networks
# Extra libraries for other ROS dependencies
pip3 install --no-cache-dir \
                  black \
                  flake8 \
                  pillow==9.2 \
                  wget \
                  colorama \
                  simple-parsing \
                  kornia \
                  pytest \
                  scipy \
                  scikit-image \
                  scikit-learn \
                  seaborn \
                  pandas \
                  fast_slic

if [[ "$JETPACK_VERSION" != "" ]]; then
# Stuff below only applies to Jetson
# Environment variables to make scikit-learn work
# If not added, it will raise an error:
#   ImportError: /usr/local/lib/python3.8/dist-packages/skimage/_shared/../../scikit_image.libs/libgomp-d22c30c5.so.1.0.0: cannot allocate memory in static TLS block
#   It seems that scikit-image has not been built correctly.
#
#   Your install of scikit-image appears to be broken.
#   Try re-installing the package following the instructions at:
#   https://scikit-image.org/docs/stable/install.html 
#
     export LD_PRELOAD=/usr/local/lib/python3.8/dist-packages/skimage/_shared/../../scikit_image.libs/libgomp-d22c30c5.so.1.0.0
fi

# Other learning packages
pip3 install --no-cache-dir neptune-client[optuna] \
                            hydra-core \
                            pytorch_lightning==1.6.5

# Other dependencies
pip3 install --no-cache-dir \
            git+https://github.com/lucasb-eyer/pydensecrf.git#egg=pydensecrf \
            git+https://github.com/mmattamala/liegroups#egg=liegroups \
            git+https://github.com/leggedrobotics/stego.git#egg=stego \
            git+https://github.com/JonasFrey96/pytorch_pwc.git#egg=pytorch_pwc==0.0.1
