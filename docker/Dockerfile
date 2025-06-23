# Dockerfile
FROM ros:humble
WORKDIR /root
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-dev \
    python3-colcon-common-extensions \
    python3-yaml \
    python3-setuptools \
    python3-pyproj \
    python3-empy \
    ros-humble-rclpy \
    ros-humble-sensor-msgs \
    ros-humble-geometry-msgs \
    ros-humble-std-msgs \
    && rm -rf /var/lib/apt/lists/*

RUN pip3 install --no-cache-dir numpy scipy pandas pyyaml
RUN mkdir -p /root/ros2_ws/src
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
CMD ["/bin/bash"]
