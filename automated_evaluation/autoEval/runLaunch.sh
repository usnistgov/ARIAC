#!/bin/bash
cd ~/ariac_ws
source install/setup.bash
#ros2 launch ariac_gazebo $1
echo "ros2 launch ariac_gazebo $1 competitor_pkg:=$2"
ros2 launch ariac_gazebo $1 competitor_pkg:=$2
