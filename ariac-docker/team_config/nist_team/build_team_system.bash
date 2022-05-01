#!/usr/bin/env bash

YELLOW='\033[0;33m'
NOCOLOR='\033[0m'

# Prepare ROS
echo -e "${YELLOW}Sourcing ROS${NOCOLOR}"
source /opt/ros/${ROS_DISTRO}/setup.bash
echo -e "${YELLOW}Sourcing ARIAC${NOCOLOR}"
source /home/ariac-user/ariac_ws/devel/setup.bash


# Install the necessary dependencies for getting the team's source code
# Note: there is no need to use `sudo`.
# apt-get update
# apt-get install -y wget unzip


# Create a catkin workspace
mkdir -p ~/nist_team_ws/src

# Fetch the source code for our team's code
# cd /tmp
# wget https://bitbucket.org/zeidk/ariac2020/get/master.zip
# unzip master.zip
# mv */ariac_example ~/my_team_ws/src

# Fetch the source code for our team's code
cd ~/nist_team_ws/src
git clone https://github.com/zeidk/nist_team.git

cd ~/nist_team_ws
catkin_make

# rm /tmp/master.zip
