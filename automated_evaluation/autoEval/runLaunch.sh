#!/bin/bash
source install/setup.bash
#ros2 launch ariac_gazebo $1
argv=("$@")
echo $2
echo "TEST"
for (( c=1; c<$#; c++ ))
do
        echo "ros2 launch ariac_gazebo $1 competitor_pkg:=${argv[c]}"
        ros2 launch ariac_gazebo $1 competitor_pkg:=${argv[c]}
done
