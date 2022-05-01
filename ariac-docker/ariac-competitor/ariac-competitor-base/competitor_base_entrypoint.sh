#!/bin/bash
set -e

# first, execute overriden entrypoint from gazebo
#source "/usr/share/gazebo/setup.sh"

# setup ros environment.
source "/opt/ros/melodic/setup.bash"

# setup ariac environment
source "/home/ariac-user/ariac_ws/devel/setup.bash"
#source "/opt/ros/melodic/etc/catkin/profile.d/99_osrf_gear_setup.sh"
echo "ariac-competitor-base entrypoint executed"

export NO_AT_BRIDGE=1
source ariac_ws/devel/setup.bash
# run gear
# TODO: optionally disable this so a gzclient can be run on the host for development.
#export GAZEBO_IP=127.0.0.1
#export GAZEBO_IP_WHITE_LIST=127.0.0.1

exec "$@"
