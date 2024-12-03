#!/usr/bin/env bash
IMAGE_NAME="aica-technology/zmq-simulator"
IMAGE_TAG="latest"
CONTAINER_NAME="aica-technology-zmq-simulator-runtime"
USERNAME="ros2"

FWD_ARGS+=(--net host)
FWD_ARGS+=(--volume="${PWD}/pybullet_zmq:/home/ros2/pybullet_zmq/pybullet_zmq:rw")
FWD_ARGS+=()

# Check for nvidia gpu
USE_NVIDIA_TOOLKIT=false 
if command -v nvidia-smi &> /dev/null; then
    if nvidia-smi --list-gpus | grep -q "GPU"; then
        echo "Using Nvidia GPU"
        RUN_FLAGS+=(--gpus all)
    fi
fi

RUN_FLAGS+=(-u "${USERNAME}")
RUN_FLAGS+=(-e DISPLAY="${DISPLAY}")
RUN_FLAGS+=(-e XAUTHORITY="${XAUTHORITY}")
RUN_FLAGS+=(-v /tmp/.X11-unix:/tmp/.X11-unix:rw)
RUN_FLAGS+=(--device=/dev/dri:/dev/dri)

RUN_CMD=(zmq-simulator)

docker run -it --rm \
"${RUN_FLAGS[@]}" \
--name "${CONTAINER_NAME}" \
"${FWD_ARGS[@]}" \
"${IMAGE_NAME}:${IMAGE_TAG}" \
"${RUN_CMD[@]}"

# -v "$(pwd)"/path/to/host_folder:/path/to/docker_folder
# aica-docker interactive aica-technology/zmq-simulator --net host --no-hostname  -v "$(pwd)"/pybullet_zmq:/home/ros2/pybullet_zmq/pybullet_zmq 
