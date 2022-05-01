#!/usr/bin/env bash

# This script is used to create and run a docker container from an image
# (usually from a built src-cloudsim image).
# The script expects 3 arguments:
# --- 1) The target container name.
# --- 2) The name of the docker image from which to create and run the container.
# --- 3) Optional extra arguments for docker run command. E.g., some extra -v options
# --- 4) An optional command to execute in the run container. E.g. /bin/bash
# Example command line:
# ./run_container.bash test ariac-server "-v logs:/home/cloudsim/gazebo-logs" /bin/bash

set -x

CONTAINER=$1
IMAGE_NAME=$2
DOCKER_EXTRA_ARGS=$3
COMMAND=$4

NETWORK="ariac-network"
IP="172.18.0.22"

# XAUTH=/tmp/.docker.xauth
# xauth nlist :0 | sed -e 's/^..../ffff/' | xauth -f $XAUTH nmerge -
if [ ! -f /tmp/.docker.xauth ]
then
  export XAUTH=/tmp/.docker.xauth
  xauth nlist :0 | sed -e 's/^..../ffff/' | xauth -f $XAUTH nmerge -
fi

# Use lspci to check for the presence of an nvidia graphics card
# TODO: re-enable nvidia support optionally.
#has_nvidia=`lspci | grep -i nvidia | wc -l`
has_nvidia=0

# Set docker gpu parameters
if [ ${has_nvidia} -gt 0 ]
then
  # check if nvidia-modprobe is installed
  if ! which nvidia-modprobe > /dev/null
  then
    echo nvidia-docker-plugin requires nvidia-modprobe
    echo please install nvidia-modprobe
    exit -1
  fi
  # check if nvidia-docker-plugin is installed
  if curl -s http://localhost:3476/docker/cli > /dev/null
  then
    DOCKER_GPU_PARAMS=" $(curl -s http://localhost:3476/docker/cli)"
    DISPLAY_PARAMS=" -v /tmp/.X11-unix:/tmp/.X11-unix -e DISPLAY=unix$DISPLAY"
  else
    echo nvidia-docker-plugin not responding on http://localhost:3476/docker/cli
    echo please install nvidia-docker-plugin
    echo https://github.com/NVIDIA/nvidia-docker/wiki/Installation
    exit -1
  fi
else
  DOCKER_GPU_PARAMS=""
  DISPLAY_PARAMS=""
fi

DISPLAY="${DISPLAY:-:0}"

xhost +

docker run --rm --name ${CONTAINER} \
  -e XAUTHORITY=/tmp/.docker.xauth \
  -e ROS_IP=${IP} \
  -e ROS_MASTER_URI=http://${IP}:11311 \
  -e NO_AT_BRIDGE=1 \
  --ip ${IP} \
  --net ${NETWORK} \
  -v "/etc/localtime:/etc/localtime:ro" \
  -v "/tmp/.docker.xauth:/tmp/.docker.xauth" \
  -v /dev/log:/dev/log \
  ${DOCKER_EXTRA_ARGS} \
  ${DOCKER_GPU_PARAMS} \
  ${DISPLAY_PARAMS} \
  ${IMAGE_NAME} \
  ${COMMAND}
