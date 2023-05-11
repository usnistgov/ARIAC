#!/bin/bash

cd ../home/ubuntu/scripts/

chmod +x run_trial.py

source /opt/ros/galactic/setup.bash
source /home/ubuntu/ariac_ws/install/setup.bash

python3 run_trial.py $1 $2