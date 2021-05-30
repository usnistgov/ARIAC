#!/bin/bash

sleep 3
echo "...Start the competition"
rosservice call /ariac/start_competition
sleep 10
echo "...Spawn the green pump in the briefcase at as2"
rosrun gazebo_ros spawn_model -sdf -x 0.032085 -y -0.152835 -z 0.28 -R 0 -P 0 -Y 0 -file `rospack find nist_gear`/models/assembly_pump_green_ariac/model.sdf -reference_frame station2::briefcase_2::briefcase_2::briefcase -model assembly_pump_green_10
sleep 3
echo "...Spawn the red pump in the briefcase at as4"
rosrun gazebo_ros spawn_model -sdf -x 0.032085 -y -0.152835 -z 0.28 -R 0 -P 0 -Y 0 -file `rospack find nist_gear`/models/assembly_pump_red_ariac/model.sdf -reference_frame station4::briefcase_4::briefcase_4::briefcase -model assembly_pump_red_10
echo "...Spawn the red battery in the briefcase at as4"
rosrun gazebo_ros spawn_model -sdf -x -0.032465 -y 0.174845 -z 0.28 -R 0 -P 0 -Y 0 -file `rospack find nist_gear`/models/assembly_battery_red_ariac/model.sdf -reference_frame station4::briefcase_4::briefcase_4::briefcase -model assembly_battery_red_10
sleep 3
echo "...Submit shipment at as4"
rosservice call /ariac/as4/submit_shipment "order_1_assembly_shipment_0"
echo "...Spawn the green battery in the briefcase at as2"
rosrun gazebo_ros spawn_model -sdf -x -0.032465 -y 0.174845 -z 0.28 -R 0 -P 0 -Y 0 -file `rospack find nist_gear`/models/assembly_battery_green_ariac/model.sdf -reference_frame station2::briefcase_2::briefcase_2::briefcase -model assembly_battery_green_10
sleep 3
echo "...Submit shipment at as2"
rosservice call /ariac/as2/submit_shipment "order_0_assembly_shipment_0"

# echo "...Spawn products in agv4"
# rosrun gazebo_ros spawn_model -sdf -x -0.15 -y -0.1 -z 0.05 -R 3.14 -P 0 -Y 0.78 -file `rospack find nist_gear`/models/assembly_pump_blue_ariac/model.sdf -reference_frame agv4::kit_tray_4::kit_tray_4::tray -model assembly_pump_blue_10
# rosrun gazebo_ros spawn_model -sdf -x 0.1 -y 0.1 -z 0.05 -R 0 -P 0 -Y 0 -file `rospack find nist_gear`/models/assembly_battery_blue_ariac/model.sdf -reference_frame agv4::kit_tray_4::kit_tray_4::tray -model assembly_battery_blue_10
# sleep 3
# echo "...Submit AGV3"
# rosservice call /ariac/agv3/submit_shipment "as3" "order_0_kitting_shipment_0"
# sleep 5
# echo "...Submit AGV4"
# rosservice call /ariac/agv4/submit_shipment "as3" "order_1_kitting_shipment_0"

# rosservice call /ariac/as2/submit_shipment "order_0_assembly_shipment_0"

