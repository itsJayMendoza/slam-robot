FROM ros:humble
RUN apt-get update && apt-get install -y --no-install-recommends \
      python3-colcon-common-extensions python3-pytest python3-scapy \
      python3-numpy python3-scipy python3-pip \
    && rm -rf /var/lib/apt/lists/*
# pykrige for Ordinary Kriging; pinned so pip keeps the apt numpy/scipy that ROS uses.
RUN pip3 install --no-cache-dir "pykrige==1.7.2"
WORKDIR /ws
