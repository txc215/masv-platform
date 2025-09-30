FROM ros:humble-ros-base

ENV DEBIAN_FRONTEND=noninteractive
WORKDIR /root

RUN echo 'Acquire::Retries "3";' > /etc/apt/apt.conf.d/80-retries

RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-pip \
    python3-colcon-common-extensions \
    build-essential \
 && rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y --no-install-recommends \
      python3-pytest \
      ros-humble-launch-testing \
 && rm -rf /var/lib/apt/lists/*

RUN pip3 install --no-cache-dir \
    numpy scipy pandas pyyaml \
    onnx onnxruntime

RUN pip3 install --no-cache-dir matplotlib

RUN pip3 install --no-cache-dir flask 

RUN mkdir -p /root/ros2_ws/src

RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

CMD ["/bin/bash"]
