FROM ros:humble
RUN apt-get update && apt-get install -y --no-install-recommends \
      python3-colcon-common-extensions python3-pytest python3-scapy \
      python3-numpy python3-scipy \
    && rm -rf /var/lib/apt/lists/*
WORKDIR /ws
