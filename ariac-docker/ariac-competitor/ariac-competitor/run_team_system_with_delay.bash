#!/usr/bin/env bash

YELLOW='\033[0;33m'
NOCOLOR='\033[0m'
echo -e "${YELLOW}Running team system with delay${NOCOLOR}"

echo "Waiting for ROS master"
. /opt/ros/*/setup.bash
until rostopic list ; do sleep 1; done

# Wait for the ARIAC server to come up
sleep 20

# Run the team's system
echo "Running team's system"
/run_team_system.bash

echo "Team's system finished running, ending competition"
rosservice call /ariac/end_competition
echo "Competition ended"
