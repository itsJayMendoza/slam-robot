#!/usr/bin/env bash
# Run a command in a ros:humble container with this workspace mounted at /ws.
# Humble outputs go to build_humble/ install_humble/ log_humble/ so they never
# clash with the Jazzy sim build in build/ install/ log/.
# Usage: docker/run.sh [command...]     (default: interactive bash)
set -e
WS="$(cd "$(dirname "$0")/.." && pwd)"
docker image inspect slam-robot-humble >/dev/null 2>&1 || docker build -t slam-robot-humble "$WS/docker"
TTY=-i; [ -t 0 ] && TTY=-it
exec docker run --rm $TTY --net=host --user "$(id -u):$(id -g)" -e HOME=/tmp \
  -v "$WS":/ws slam-robot-humble \
  bash -lc "source /opt/ros/humble/setup.bash; [ -f install_humble/setup.bash ] && source install_humble/setup.bash; ${*:-bash}"
