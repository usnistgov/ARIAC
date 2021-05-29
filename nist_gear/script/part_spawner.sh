#!/bin/bash

# # spawn a part in the tray on agv1
# rosrun gazebo_ros spawn_model -sdf -x 0.0 -y -0.2 -z 0.05 -R 3.14159 -P 0 -Y 0 -file `rospack find nist_gear`/models/assembly_pump_blue_ariac/model.sdf -reference_frame agv1::kit_tray_1::kit_tray_1::tray -model assembly_pump_blue_10

# # spawn a part on the tray for the assembly robot
# rosrun gazebo_ros spawn_model -sdf -x 0.0 -y 0.0 -z 0.05 -R 3.14159 -P 0 -Y 0 -file `rospack find nist_gear`/models/assembly_pump_blue_ariac/model.sdf -reference_frame gantry::torso_tray -model assembly_pump_blue_11

# # spawn a part in the briefcase located at assembly station 1 
# rosrun gazebo_ros spawn_model -sdf -x 0.032085 -y -0.152835 -z 0.28 -R 3.14159 -P 0 -Y 0 -file `rospack find nist_gear`/models/assembly_pump_blue_ariac/model.sdf -reference_frame station1::briefcase_1::briefcase_1::briefcase -model assembly_pump_blue_12


#finals_practice1
rosrun gazebo_ros spawn_model -sdf -x 0.1 -y 0.1 -z 0.05 -R 0 -P 0 -Y 0 -file `rospack find nist_gear`/models/assembly_regulator_green_ariac/model.sdf -reference_frame agv3::kit_tray_3::kit_tray_3::tray -model assembly_regulator_red_10
rosrun gazebo_ros spawn_model -sdf -x -0.1 -y -0.1 -z 0.05 -R 0 -P 0 -Y 0.78 -file `rospack find nist_gear`/models/assembly_sensor_blue_ariac/model.sdf -reference_frame agv3::kit_tray_3::kit_tray_3::tray -model assembly_sensor_blue_10

rosrun gazebo_ros spawn_model -sdf -x -0.15 -y -0.1 -z 0.05 -R 3.14 -P 0 -Y 0.78 -file `rospack find nist_gear`/models/assembly_pump_blue_ariac/model.sdf -reference_frame agv4::kit_tray_4::kit_tray_4::tray -model assembly_pump_blue_10
rosrun gazebo_ros spawn_model -sdf -x 0.1 -y 0.1 -z 0.05 -R 0 -P 0 -Y 0 -file `rospack find nist_gear`/models/assembly_battery_blue_ariac/model.sdf -reference_frame agv4::kit_tray_4::kit_tray_4::tray -model assembly_battery_blue_10

# rosrun gazebo_ros spawn_model -sdf -x 0.1 -y 0.1 -z 0.05 -R 0 -P 0 -Y 0.78 -file `rospack find nist_gear`/models/assembly_battery_red_ariac/model.sdf -reference_frame agv4::kit_tray_4::kit_tray_4::tray -model assembly_battery_red_40
# rosrun gazebo_ros spawn_model -sdf -x -0.15 -y -0.1 -z 0.05 -R 0 -P 0 -Y 0 -file `rospack find nist_gear`/models/assembly_pump_green_ariac/model.sdf -reference_frame agv4::kit_tray_4::kit_tray_4::tray -model assembly_pump_green_40
