#!/bin/bash
# linux/ubuntu: x11 direkt durchreichen, kein vcxsrv nötig
set -e

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE="$PROJECT_ROOT/workspace"

mkdir -p "$WORKSPACE/src"

# x11 access für root-user im container
xhost +local:docker > /dev/null 2>&1 || true

docker run -it --rm \
    --name maze_robotik \
    --net=host \
    --shm-size=2g \
    --ipc=host \
    -e DISPLAY="$DISPLAY" \
    -e QT_X11_NO_MITSHM=1 \
    -e XDG_RUNTIME_DIR=/tmp \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v "$WORKSPACE:/root/ros2_ws" \
    --device=/dev/dri:/dev/dri \
    maze-robotik:latest
