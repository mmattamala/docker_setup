ARG BASE_IMAGE=nvidia/cuda:11.8.0-cudnn8-devel-ubuntu20.04
FROM ${BASE_IMAGE}

# Input script
ARG SCRIPT=""
# Other arguments used as env variables
ARG UBUNTU_VERSION="ubuntu20.04"
ARG ROS_VERSION="noetic"
ARG WITH_CUDA="false"
ARG CUDA_VERSION=""
ARG CUDA_ARCH_BIN=""
ARG JETPACK_VERSION=""
# Define environment variables
ENV UBUNTU_VERSION=${UBUNTU_VERSION}
ENV ROS_VERSION=${ROS_VERSION}
ENV WITH_CUDA=${WITH_CUDA}
ENV CUDA_VERSION=${CUDA_VERSION}
ENV CUDA_ARCH_BIN=${CUDA_ARCH_BIN}
ENV JETPACK_VERSION=${JETPACK_VERSION}

# Labels
LABEL maintainer="Matias Mattamala"
LABEL contact="matias@robots.ox.ac.uk"
LABEL description="Image with personal development setup"
LABEL example_usage="docker run --cidfile /tmp/jetson_docker.cid -it --rm --net=host --runtime nvidia -e DISPLAY=$DISPLAY -v /tmp/.X11-unix/:/tmp/.X11-unix -v ${YOUR_GIT_WS}:/root/git -v ${YOUR_CATKIN_WS}:/root/catkin_ws/ mmattamala/personal-setup:${UBUNTU_VERSION}"

# Terminal color
SHELL ["/bin/bash", "--login", "-c"]
ENV TERM=xterm-256color

# To avoid tzdata asking for geographic location...
ARG DEBIAN_FRONTEND=noninteractive
ENV DEBIAN_frontend noninteractive

# ==
# Install script
# ==
COPY stages/${SCRIPT} /root/${SCRIPT}
RUN chmod +x /root/${SCRIPT}
RUN /root/${SCRIPT} && rm /root/${SCRIPT}

# ==
# Remove cache and extra files
# ==
RUN rm -rf /var/lib/apt/lists/* && apt-get clean

# ==
# Copy helper scripts (.bashrc)
#==
# Bash aliases
COPY .bash_aliases /root/.bash_aliases

# Custom entrypoint
COPY entrypoints/dummy.sh /custom_entrypoint.sh
RUN chmod +x /custom_entrypoint.sh

# ==
# Setup entrypoint
# ==
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
WORKDIR /root/
