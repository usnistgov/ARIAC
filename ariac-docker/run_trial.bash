#!/usr/bin/env bash

set -e

TEAM_NAME=$1
TRIAL_NAME=$2

# Constants.
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[0;33m'
NOCOLOR='\033[0m'

CONTAINER_NAME=ariac-server-system

# Create the directory that logs will be copied into. Since the userid of the user in the container
# might different to the userid of the user running this script, we change it to be public-writable.
HOST_LOG_DIR=`pwd`/logs/${TEAM_NAME}/${TRIAL_NAME}
echo "${YELLOW}...Creating directory: ${HOST_LOG_DIR}${NOCOLOR}"



if [ -d "$HOST_LOG_DIR" ]; then
  echo -e "${YELLOW}Wrn: Destination folder $HOST_LOG_DIR already exists. Data might be"\
          "overwritten${NOCOLOR}"
fi

echo -e "${YELLOW}...Creating $HOST_LOG_DIR directory${NOCOLOR}"
mkdir -p $HOST_LOG_DIR

# TODO: don't rely on script being run in the root directory
# TODO: error checking for case when files can't be found
TEAM_CONFIG_DIR=`pwd`/team_config/${TEAM_NAME}
echo -e "${YELLOW}...Using team config: ${TEAM_CONFIG_DIR}/team_config.yaml${NOCOLOR}"
COMP_CONFIG_DIR=`pwd`/trial_config
echo -e "${YELLOW}...Using comp config: ${COMP_CONFIG_DIR}/${TRIAL_NAME}.yaml${NOCOLOR}"

ROS_DISTRO=melodic

LOG_DIR=/ariac/logs

# Ensure any previous containers are killed and removed.
echo -e "${YELLOW}...Killing any previous containers${NOCOLOR}"
./kill_ariac_containers.bash

# Create the network for the containers to talk to each other.
echo -e "${YELLOW}...Creating network for containers${NOCOLOR}"
./ariac-competitor/ariac_network.bash

# Start the competitors container and let it run in the background.
echo -e "${YELLOW}...Starting competition container in the background${NOCOLOR}"
COMPETITOR_IMAGE_NAME="ariac-competitor-${TEAM_NAME}"
./ariac-competitor/run_competitor_container.bash ${COMPETITOR_IMAGE_NAME} "/run_team_system_with_delay.bash" &

# Start the competition server. When the trial ends, the container will be killed.
# The trial may end because of time-out, because of completion, or because the user called the
# /ariac/end_competition service.
echo -e "${YELLOW}...Starting the competition server${NOCOLOR}"
./ariac-server/run_container.bash ${CONTAINER_NAME} zeidk/ariac6-server-${ROS_DISTRO}:latest \
  "-v ${TEAM_CONFIG_DIR}:/team_config \
  -v ${COMP_CONFIG_DIR}:/ariac/trial_config \
  -v ${HOST_LOG_DIR}:${LOG_DIR} \
  -e ARIAC_EXIT_ON_COMPLETION=1" \
  "/run_ariac_task.sh /ariac/trial_config/${TRIAL_NAME}.yaml /team_config/team_config.yaml ${LOG_DIR}"
#
# # Copy the ROS log files from the competitor's container.
echo -e "${YELLOW}...Copying ROS log files from competitor container${NOCOLOR}"
docker cp --follow-link ariac-competitor-${TEAM_NAME}-system:/root/.ros/log $HOST_LOG_DIR/ros-competitor

./kill_ariac_containers.bash

exit 0
