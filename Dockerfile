ARG BASE_IMAGE=nvidia/cuda:11.8.0-cudnn8-devel-ubuntu20.04
FROM ${BASE_IMAGE}

ARG BASE_IMAGE
ARG UBUNTU_VERSION=20.04
ARG ROS_VERSION=noetic
ARG WITH_CUDA=true
ARG CUDA_VERSION=""
ARG CUDA_ARCH_BIN=""

# Print general options
RUN echo -e "Building Dockerfile \
          \n BASE_IMAGE=$BASE_IMAGE \ 
          \n UBUNTU_VERSION=$UBUNTU_VERSION \
          \n ROS_VERSION=$ROS_VERSION \
          \n WITH_CUDA=$WITH_CUDA \
          \n CUDA_VERSION=$CUDA_VERSION \
          \n CUDA_ARCH_BIN=$CUDA_ARCH_BIN"

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
# Install base tools
# ==
COPY scripts/install_base.sh /home/install_base.sh
RUN chmod +x home/install_base.sh
RUN /home/install_base.sh && rm /home/install_base.sh

# ==
# Install OpenCV (with CUDA if required)
# ==
COPY scripts/install_opencv.sh /home/install_opencv.sh
RUN chmod +x /home/install_opencv.sh
RUN /home/install_opencv.sh --ros-version=${ROS_VERSION} --cuda=${WITH_CUDA} --cuda-arch-bin=${CUDA_ARCH_BIN} && rm /home/install_opencv.sh

# ==
# Install ROS
# ==
COPY scripts/install_ros.sh /home/install_ros.sh
RUN chmod +x /home/install_ros.sh
RUN /home/install_ros.sh --version=${ROS_VERSION} && rm /home/install_ros.sh

# ==
# Install ML libraries
# ==
COPY scripts/install_ml.sh /home/install_ml.sh
RUN chmod +x /home/install_ml.sh
RUN /home/install_ml.sh && rm /home/install_ml.sh

# ==
# Setup entrypoint
# ==
COPY entrypoint.sh /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
WORKDIR /root/
