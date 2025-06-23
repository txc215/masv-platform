FROM ros:humble-ros-base

# basic
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

# option: install dependance 
RUN pip3 install -U numpy

# Set work directory 
WORKDIR /root/ros2_ws


RUN useradd -m dev
USER dev
