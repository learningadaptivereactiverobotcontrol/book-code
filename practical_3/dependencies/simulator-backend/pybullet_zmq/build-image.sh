#!/usr/bin/env bash
ROS_VERSION=humble

IMAGE_NAME=aica-technology/zmq-simulator
IMAGE_TAG=latest

SERVE_REMOTE=false
REMOTE_SSH_PORT=7770

HELP_MESSAGE="Usage: build-server.sh [-r] [-v] [-s]
Options:
  -r, --rebuild                   Rebuild the image using the docker
                                  --no-cache option.

  -v, --verbose                   Use the verbose option during the building
                                  process.

  -h, --help                      Show this help message.
"

BUILD_FLAGS=()
while [[ $# -gt 0 ]]; do
  opt="$1"
  case $opt in
    -r|--rebuild) BUILD_FLAGS+=(--no-cache) ; shift ;;
    -v|--verbose) BUILD_FLAGS+=(--progress=plain) ; shift ;;
    -h|--help) echo "${HELP_MESSAGE}" ; exit 0 ;;
    *) echo 'Error in command line parsing' >&2
       echo -e "\n${HELP_MESSAGE}"
       exit 1
  esac
done

if [[ "$OSTYPE" != "darwin"* ]]; then
  BUILD_FLAGS+=(--ssh default="${SSH_AUTH_SOCK}")
else
  BUILD_FLAGS+=(--ssh default="$HOME/.ssh/id_rsa")
fi

BUILD_FLAGS+=(--build-arg ROS_VERSION="${ROS_VERSION}")
BUILD_FLAGS+=(-t "${IMAGE_NAME}:${IMAGE_TAG}")

docker pull ghcr.io/aica-technology/ros2-control-libraries:"${ROS_VERSION}" || exit 1
DOCKER_BUILDKIT=1 docker build "${BUILD_FLAGS[@]}" --file ./Dockerfile .. || exit 1

