#!/bin/bash
set -e

# first, execute overriden entrypoint from gazebo
source "/usr/share/gazebo/setup.sh"

# setup ros environment.
source "/opt/ros/melodic/setup.bash" > /dev/null

# setup ariac environment
source "/home/ariac/ariac_ws/devel/setup.bash"
#source "/opt/ros/melodic/etc/catkin/profile.d/99_osrf_gear_setup.sh"
echo "ariac-server entrypoint executed"

# run gear
# TODO: optionally disable this so a gzclient can be run on the host for development.
export GAZEBO_IP=127.0.0.1
export GAZEBO_IP_WHITE_LIST=127.0.0.1

exec "$@"
