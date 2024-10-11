#!/bin/bash

# Set default variables
LOCAL_BASE_IMAGE=false
BASE_TAG=humble
BUILD_LOCAL=false  # New variable to control local build or pull from GHCR

IMAGE_NAME=epfl-lasa/larrc/practical_3_sim
IMAGE_TAG=latest

REMOTE_SSH_PORT=1003
SERVE_REMOTE=false

HELP_MESSAGE="Usage: build.sh [-p] [-r]
Options:
  -d, --development      Only target the dependencies layer to prevent
                         sources from being built or tested

  -r, --rebuild          Rebuild the image(s) using the docker
                         --no-cache option

  -v, --verbose          Use the verbose option during the building
                         process

  -l, --local            Build the image locally instead of pulling from GHCR
"

# Store the script's directory
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="${SCRIPT_DIR}/.."  # Assuming 'docker' is located in the project root

# Set build flags
BUILD_FLAGS=(--build-arg BASE_TAG="${BASE_TAG}")

# Parse command-line options
while [[ $# -gt 0 ]]; do
  opt="$1"
  case $opt in
    -d|--development) BUILD_FLAGS+=(--target dependencies) ; IMAGE_TAG=development ; shift ;;
    -r|--rebuild) BUILD_FLAGS+=(--no-cache) ; shift ;;
    -v|--verbose) BUILD_FLAGS+=(--progress=plain) ; shift ;;
    -l|--local) BUILD_LOCAL=true ; shift ;;  # New option to build locally
    -h|--help) echo "${HELP_MESSAGE}" ; exit 0 ;;
    *) echo 'Error in command line parsing' >&2
       echo -e "\n${HELP_MESSAGE}"
       exit 1
  esac
done

# Pull base image if needed
if [ "${LOCAL_BASE_IMAGE}" == true ]; then
  BUILD_FLAGS+=(--build-arg BASE_IMAGE=aica-technology/ros2-modulo)
else
  docker pull ghcr.io/aica-technology/ros2-modulo:"${BASE_TAG}"
fi

# Determine build context based on the current directory
if [[ "$(basename "$PWD")" == "docker" ]]; then
  BUILD_CONTEXT="${PROJECT_ROOT}"
else
  BUILD_CONTEXT="."
fi

# Build the image locally or pull from GHCR depending on BUILD_LOCAL
if [ "${BUILD_LOCAL}" == true ]; then
  echo "Building image locally..."
  DOCKER_BUILDKIT=1 docker build -t "${IMAGE_NAME}:${IMAGE_TAG}" "${BUILD_FLAGS[@]}" -f "${SCRIPT_DIR}/Dockerfile" "${BUILD_CONTEXT}" || exit 1
else
  echo "Pulling image from GHCR..."
  docker pull ghcr.io/"${IMAGE_NAME}:${IMAGE_TAG}" || exit 1
# Explicitly tag the pulled image with the correct image name and tag
  docker tag ghcr.io/"${IMAGE_NAME}:${IMAGE_TAG}" "${IMAGE_NAME}:${IMAGE_TAG}"
fi


