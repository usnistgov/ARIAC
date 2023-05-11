#!/bin/bash

cd ../home/ubuntu/scripts/

chmod +x build_competitor_code.py

rm -r /home/ubuntu/ariac_ws/src/ariac/ariac_gazebo/config/trials
mv /home/ubuntu/trials /home/ubuntu/ariac_ws/src/ariac/ariac_gazebo/config/

source /opt/ros/galactic/setup.bash
python3 build_competitor_code.py $1