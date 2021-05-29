#!/bin/bash

sleep 3
echo "...Start the competition"
rosservice call /ariac/start_competition
sleep 10
echo "...Spawn products in agv1"
rosrun gazebo_ros spawn_model -sdf -x 0.1 -y 0.1 -z 0.05 -R 3.14159 -P 0 -Y 0 -file `rospack find nist_gear`/models/assembly_pump_red_ariac/model.sdf -reference_frame agv1::kit_tray_1::kit_tray_1::tray -model assembly_pump_red_20
rosrun gazebo_ros spawn_model -sdf -x 0.0 -y -0.1 -z 0.05 -R 0 -P 0 -Y 0 -file `rospack find nist_gear`/models/assembly_battery_red_ariac/model.sdf -reference_frame agv1::kit_tray_1::kit_tray_1::tray -model assembly_battery_red_20
sleep 3
echo "...Spawn products in agv2"
rosrun gazebo_ros spawn_model -sdf -x 0.1 -y 0.1 -z 0.05 -R 3.14159 -P 0 -Y 0 -file `rospack find nist_gear`/models/assembly_pump_red_ariac/model.sdf -reference_frame agv2::kit_tray_2::kit_tray_2::tray -model assembly_pump_red_30
rosrun gazebo_ros spawn_model -sdf -x 0.0 -y -0.1 -z 0.05 -R 0 -P 0 -Y 0 -file `rospack find nist_gear`/models/assembly_battery_red_ariac/model.sdf -reference_frame agv2::kit_tray_2::kit_tray_2::tray -model assembly_battery_red_30
sleep 4
echo "...Submit agv1"
rosservice call /ariac/agv1/submit_shipment "as2" "order_0_kitting_shipment_0"
echo "...Submit agv2"
rosservice call /ariac/agv2/submit_shipment "as2" "order_0_kitting_shipment_1"
sleep 10
echo "...Spawn the red pump in the briefcase at as2"
rosrun gazebo_ros spawn_model -sdf -x 0.032085 -y -0.152835 -z 0.28 -R 3.14159 -P 0 -Y 0 -file `rospack find nist_gear`/models/assembly_pump_red_ariac/model.sdf -reference_frame station2::briefcase_2::briefcase_2::briefcase -model assembly_pump_red_35
echo "...Spawn the red battery in the briefcase at as2"
rosrun gazebo_ros spawn_model -sdf -x -0.032465 -y 0.174845 -z 0.28 -R 0 -P 0 -Y 0 -file `rospack find nist_gear`/models/assembly_battery_red_ariac/model.sdf -reference_frame station2::briefcase_2::briefcase_2::briefcase -model assembly_battery_red_35
sleep 3
echo "...Submit shipment at as2"
rosservice call /ariac/as2/submit_shipment "order_1_assembly_shipment_0"