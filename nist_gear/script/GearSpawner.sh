#!/bin/bash

# spawn a part directly on kit_tray_1
# rosrun gazebo_ros spawn_model -sdf -x 0.0 -y -0.2 -z 0.05 -R 3.14159 -P 0 -Y 0 -file `rospack find nist_gear`/models/assembly_pump_blue_ariac/model.sdf -reference_frame agv1::kit_tray_1::kit_tray_1::tray -model assembly_pump_blue_10
# spawn a part directly on kit_tray_2
# rosrun gazebo_ros spawn_model -sdf -x 0.0 -y -0.2 -z 0.05 -R 3.14159 -P 0 -Y 0 -file `rospack find nist_gear`/models/assembly_pump_blue_ariac/model.sdf -reference_frame agv2::kit_tray_2::kit_tray_2::tray -model assembly_pump_blue_10
# spawn a part directly on kit_tray_3
# rosrun gazebo_ros spawn_model -sdf -x 0.0 -y -0.2 -z 0.05 -R 3.14159 -P 0 -Y 0 -file `rospack find nist_gear`/models/assembly_pump_blue_ariac/model.sdf -reference_frame agv1::kit_tray_3::kit_tray_3::tray -model assembly_pump_blue_10
# spawn a part on the tray for the assembly robot
# rosrun gazebo_ros spawn_model -sdf -x 0.0 -y 0.0 -z 0.05 -R 3.14159 -P 0 -Y 0 -file `rospack find nist_gear`/models/assembly_pump_blue_ariac/model.sdf -reference_frame gantry::torso_tray -model assembly_pump_blue_11
# # spawn a part in the briefcase located at assembly station 1 
# rosrun gazebo_ros spawn_model -sdf -x 0.032085 -y -0.152835 -z 0.28 -R 3.14159 -P 0 -Y 0 -file `rospack find nist_gear`/models/assembly_pump_blue_ariac/model.sdf -reference_frame station1::briefcase_1::briefcase_1::briefcase -model assembly_pump_blue_12


# spawn a part on a movable tray
# rosrun gazebo_ros spawn_model -sdf -x 0.1 -y -0.1 -z 0.05 -R 0 -P 0 -Y 0 -file `rospack find nist_gear`/models/assembly_pump_blue_ariac/model.sdf -reference_frame movable_tray_metal_shiny_1::movable_tray_metal_shiny_1::link -model assembly_pump_blue_10
# sleep 5
# # spawn a part on a movable tray
# rosservice call /ariac/start_competition
# sleep 2
# rosrun gazebo_ros spawn_model -sdf -x 0.1 -y -0.1 -z 0.05 -R 0 -P 0 -Y 0 -file `rospack find nist_gear`/models/assembly_sensor_red_ariac/model.sdf -reference_frame movable_tray_dark_wood_1::movable_tray_dark_wood_1::link -model assembly_sensor_red_12


# rosservice call /ariac/agv1/move_to_station as1  
# rosservice call /ariac/agv2/move_to_station as1
# rosservice call /ariac/agv3/move_to_station as3  
# rosservice call /ariac/agv3/move_to_station as4  
# sleep 10
# rosservice call /ariac/agv1/move_to_station as2  
# rosservice call /ariac/agv2/move_to_station as2  
# rosservice call /ariac/agv3/move_to_station as4  
# rosservice call /ariac/agv4/move_to_station as4  
# sleep 10
# rosservice call /ariac/agv1/move_to_station as1  
# rosservice call /ariac/agv2/move_to_station as1  
# rosservice call /ariac/agv3/move_to_station as3  
# rosservice call /ariac/agv4/move_to_station as3
# sleep 10
# rosservice call /ariac/agv1/move_to_station ks  
# rosservice call /ariac/agv2/move_to_station ks  
# rosservice call /ariac/agv3/move_to_station ks  
# rosservice call /ariac/agv4/move_to_station ks   
# sleep 10
# rosservice call /ariac/agv1/move_to_station as2  
# rosservice call /ariac/agv2/move_to_station as2  
# rosservice call /ariac/agv3/move_to_station as4  
# rosservice call /ariac/agv4/move_to_station as4  
# sleep 10
# rosservice call /ariac/agv1/move_to_station ks  
# rosservice call /ariac/agv2/move_to_station ks  
# rosservice call /ariac/agv3/move_to_station ks  
# rosservice call /ariac/agv4/move_to_station ks    
# rosservice call /ariac/kit_tray_1/lock 
# sleep 2
# rosservice call /ariac/kit_tray_1/unlock 
# sleep 2
# rosservice call /ariac/kit_tray_1/lock 
# rosrun gazebo_ros spawn_model -sdf -x -0.1 -y -0.1 -z 0.05 -R 0 -P 0 -Y 0 -file `rospack find nist_gear`/models/assembly_pump_green_ariac/model.sdf -reference_frame movable_tray_metal_shiny_1::movable_tray_metal_shiny_1::link -model assembly_pump_green_10
# rosrun gazebo_ros spawn_model -sdf -x 0.1 -y -0.1 -z 0.05 -R 0 -P 0 -Y 0 -file `rospack find nist_gear`/models/assembly_sensor_red_ariac/model.sdf -reference_frame movable_tray_dark_wood_1::movable_tray_dark_wood_1::link -model assembly_sensor_red_10


# sleep 5
# rosservice call /ariac/agv1/submit_kitting_shipment "as1" "test"

# rosrun gazebo_ros spawn_model -sdf -x 0.032085 -y -0.152835 -z 0.28 -R 3.14159 -P 0 -Y 0 -file `rospack find nist_gear`/models/assembly_pump_blue_ariac/model.sdf -reference_frame station1::briefcase_1::briefcase_1::briefcase -model assembly_pump_blue_10
# sleep 10
# rosrun gazebo_ros spawn_model -sdf -x 0.032085 -y -0.152835 -z 0.28 -R 3.14159 -P 0 -Y 0 -file `rospack find nist_gear`/models/assembly_pump_green_ariac/model.sdf -reference_frame station2::briefcase_2::briefcase_2::briefcase -model assembly_pump_green_10


# rosservice call /ariac/agv1/submit_kitting_shipment "as1" "order_0_kitting_shipment_0"


#############
# kitting_sample.yaml
#############
rosservice call /ariac/start_competition
rosrun gazebo_ros spawn_model -sdf -x -0.1 -y -0.1 -z 0.05 -R 0 -P 0 -Y 0 -file `rospack find nist_gear`/models/assembly_pump_red_ariac/model.sdf -reference_frame movable_tray_metal_shiny_1::movable_tray_metal_shiny_1::link -model assembly_pump_red_12
sleep 5
rosservice call /ariac/agv1/submit_kitting_shipment "as1" "order_0_kitting_shipment_0" 

##############
# testing the service /ariac/agvX/content
##############
# rosservice call /ariac/start_competition
# rosrun gazebo_ros spawn_model -sdf -x 0.1 -y -0.1 -z 0.05 -R 0 -P 0 -Y 0 -file `rospack find nist_gear`/models/assembly_sensor_red_ariac/model.sdf -reference_frame movable_tray_dark_wood_1::movable_tray_dark_wood_1::link -model assembly_sensor_red_10
# rosservice call /ariac/agv1/content


###################################
# test_hpo.yaml
###################################
# rosservice call /ariac/start_competition
# rosrun gazebo_ros spawn_model -sdf -x -0.032465 -y 0.174845 -z 0.18 -R 0 -P 0 -Y 3.14159 -file `rospack find nist_gear`/models/assembly_battery_red_ariac/model.sdf -reference_frame station4::briefcase_4::briefcase_4::briefcase -model assembly_battery_red_12
# sleep 5
# rosrun gazebo_ros spawn_model -sdf -x -0.032465 -y 0.174845 -z 0.18 -R 0 -P 0 -Y 0 -file `rospack find nist_gear`/models/assembly_battery_blue_ariac/model.sdf -reference_frame station1::briefcase_1::briefcase_1::briefcase -model assembly_battery_blue_12
# rosrun gazebo_ros spawn_model -sdf -x 0.032085 -y -0.152835 -z 0.28 -R 0 -P 0 -Y 0 -file `rospack find nist_gear`/models/assembly_pump_blue_ariac/model.sdf -reference_frame station1::briefcase_1::briefcase_1::briefcase -model assembly_pump_blue_12
# sleep 2.0
# rosservice call /ariac/as1/submit_assembly_shipment "order_1_assembly_shipment_0"

# sleep 5.0
# rosrun gazebo_ros spawn_model -sdf -x 0.032085 -y -0.152835 -z 0.28 -R 0 -P 0 -Y 0 -file `rospack find nist_gear`/models/assembly_pump_red_ariac/model.sdf -reference_frame station4::briefcase_4::briefcase_4::briefcase -model assembly_pump_red_12
# sleep 2.0
# rosservice call /ariac/as4/submit_assembly_shipment "order_0_assembly_shipment_0"
