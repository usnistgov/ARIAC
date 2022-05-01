#!/usr/bin/env bash

# This script is used to create and run a docker container from an image
# (usually from a built ariac-competitor image).
# The script does not expect any arguments, but may use:
# --- 1) An optional command to execute in the run container. E.g. /bin/bash
# Example command line usage:
# ./run_competitor_container.bash /bin/bash

set -e
set -x

# Constants.
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[0;33m'
NOCOLOR='\033[0m'

IMAGE_NAME=$1
COMMAND=$2

CONTAINER="${IMAGE_NAME}-system"
DOCKER_EXTRA_ARGS=""

NETWORK="ariac-network"
IP="172.18.0.20"
SERVER_IP="172.18.0.22"

echo -e "${GREEN}Starting docker container named '${CONTAINER}' with IP ${IP}...${NOCOLOR}"

xhost +

DISPLAY="${DISPLAY:-:0}"
# Keep the container around afterwards (no --rm) in case we need to copy files.
docker run --name ${CONTAINER} \
  --env="DISPLAY" \
  --env="QT_X11_NO_MITSHM=1" \
  --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
  -e ROS_IP=${IP} \
  -e ROS_MASTER_URI=http://${SERVER_IP}:11311 \
  -e NO_AT_BRIDGE=1 \
  --ip ${IP} \
  --net ${NETWORK} \
  ${IMAGE_NAME} \
  ${COMMAND} \
  "source /home/ariac-user/ariac_ws/devel/setup.bash"

