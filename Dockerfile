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


# RVIZ2 Forwarding
ENV NVIDIA_VISIBLE_DEVICES \
    ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES \
    ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics

# Setup directory
RUN mkdir -p /workplace
WORKDIR /workplace