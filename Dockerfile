#
# A development dockerfile for a ROS2 container that we will be running all of our code in.
#

FROM ros:humble

ENV ROS_DISTRO=humble
# Install C++ Package Dependencies
RUN apt update -y && sudo apt upgrade -y

# Install ROS2 Dependencies
RUN apt-get update && apt-get upgrade -y && apt-get install -y \
    ros-$ROS_DISTRO-teleop-twist-keyboard \
    ros-$ROS_DISTRO-rviz2 \
    ros-$ROS_DISTRO-rviz-common \
    ros-$ROS_DISTRO-rviz-default-plugins \
    ros-$ROS_DISTRO-rviz-visual-tools \
    ros-$ROS_DISTRO-rviz-rendering \
    ros-$ROS_DISTRO-nav2-rviz-plugins && \
    apt-get upgrade -y && \
    apt-get autoremove -y && \
    apt-get clean

# Install Build/Debug dependencies
RUN apt install libspdlog-dev -y
RUN apt install libeigen3-dev -y
RUN apt install gdb -y
RUN apt install plocate -y
RUN apt install mpich -y
RUN apt install wget -y

# CUDA Setup
ENV NVIDIA_VISIBLE_DEVICES all
ENV NVIDIA_DRIVER_CAPABILITIES compute,utility
ENV NVIDIA_REQUIRE_CUDA "cuda>=8.0"

# Install CUDA
RUN wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2204/x86_64/cuda-keyring_1.1-1_all.deb
RUN dpkg -i cuda-keyring_1.1-1_all.deb
RUN apt update
RUN apt install cuda-toolkit-12-2 -y

# RVIZ2 Forwarding
ENV NVIDIA_VISIBLE_DEVICES \
    ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES \
    ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics

ENV CUDA_HOME=/usr/local/cuda
ENV LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/cuda/lib64:/usr/local/cuda/extras/CUPTI/lib64
ENV PATH=$PATH:$CUDA_HOME/bin

# Additional ROS2 Deps
RUN apt install ros-$ROS_DISTRO-pcl-conversions -y
RUN apt install ros-$ROS_DISTRO-sensor-msgs -y
RUN apt install ros-$ROS_DISTRO-pcl-msgs -y
RUN apt remove mpich -y

# Setup directory
RUN mkdir -p /workplace
WORKDIR /workplace