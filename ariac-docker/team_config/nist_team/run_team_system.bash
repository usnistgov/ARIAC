#!/usr/bin/env bash

YELLOW='\033[0;33m'
NOCOLOR='\033[0m'
echo -e "${YELLOW}Running team system${NOCOLOR}"
. ~/nist_team_ws/devel/setup.bash

# Run the example node
echo "${YELLOW}Launching ARIAC example node${NOCOLOR}"
rosrun nist_team change_gripper_test.py
