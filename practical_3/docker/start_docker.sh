#!/bin/bash
IMAGE_NAME="epfl-lasa/larrc/practical_3_sim"
IMAGE_TAG="latest"
CONTAINER_NAME="${IMAGE_NAME//[\/.]/-}"
USERNAME="ros2"
MODE="interactive"
ROS_DOMAIN_ID=14

# Help
HELP_MESSAGE="Usage: ./start_docker.sh [interactive | connect] 
Options:
  interactive            Spin the image in the console
  connect                Connects to an active container
  "

# Argument parsing
RUN_FLAGS=()
FWD_FLAGS=()
EXEC_FLAGS=()

while [ "$#" -gt 0 ]; do
    case "$1" in
    -m | --mode)
        MODE=$2
        shift 2
        ;;
    -h | --help)
        SHOW_HELP=true
        shift 1
        ;;
    *)
        if [ -z "${MODE}" ]; then
            MODE=$1
        else
            FWD_ARGS+=("$1")
        fi
        shift 1
        ;;
    esac
done

# Store the script's directory
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="${SCRIPT_DIR}/.."  # Assuming 'docker' is located in the project root


# Handle interactive/server specific arguments
if [ "${MODE}" == "interactive" ]; then

    # Check if a conitainer with this name is already running
    if [ "$( docker container inspect -f '{{.State.Status}}' ${CONTAINER_NAME} 2>/dev/null)" == "running" ]; then
        echo "A container named ${CONTAINER_NAME} is already running. Stopping it."
        docker stop ${CONTAINER_NAME}
    fi
	
    # network for ros
    FWD_ARGS+=(--net host)
    # FWD_ARGS+=("--no-hostname")
    FWD_ARGS+=(--env ROS_DOMAIN_ID="${ROS_DOMAIN_ID}")

    ## Use nvidia if available
    if command -v nvidia-smi &> /dev/null; then
        if nvidia-smi --list-gpus | grep -q "GPU"; then
            echo "Using Nvidia GPU"
            RUN_FLAGS+=(--gpus all)
        fi
    fi

    # Volume
    FWD_ARGS+=(--volume="${PROJECT_ROOT}/matlab_bridge:/home/ros2/ros2_ws/src/matlab_bridge:rw")

    # Other
    FWD_ARGS+=("--privileged")
    FWD_ARGS+=("--ipc=host")

    RUN_FLAGS+=(-u "${USERNAME}")
    RUN_FLAGS+=(-e DISPLAY="${DISPLAY}")
    RUN_FLAGS+=(-e XAUTHORITY="${XAUTHORITY}")
    RUN_FLAGS+=(-v /tmp/.X11-unix:/tmp/.X11-unix:rw)
    RUN_FLAGS+=(--device=/dev/dri:/dev/dri)

    docker run -it --rm \
    "${RUN_FLAGS[@]}" \
    --name "${CONTAINER_NAME}" \
    "${FWD_ARGS[@]}" \
    "${IMAGE_NAME}:${IMAGE_TAG}" 
fi

# Handle interactive/server specific arguments
if [ "${MODE}" == "connect" ]; then

    EXEC_FLAGS=()
    EXEC_FLAGS+=(-u "${USERNAME}")
    EXEC_FLAGS+=(-e DISPLAY="${DISPLAY}")
    EXEC_FLAGS+=(-e XAUTHORITY="${XAUTH}")
    
    docker container exec -it "${EXEC_FLAGS[@]}" "${FWD_ARGS[@]}" "${CONTAINER_NAME}" /bin/bash

fi

# # Start docker using aica    
# aica-docker \
#     "${MODE}" \
#     "${IMAGE_NAME}:${IMAGE_TAG}" \
#     -u "${USERNAME}" \
#     -n "${CONTAINER_NAME}" \
#     ${GPU_FLAG} \
#     "${FWD_ARGS[@]}" \
