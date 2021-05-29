#!/bin/bash

sleep 3
echo "...Start the competition"
rosservice call /ariac/start_competition
sleep 3
echo "...Spawn products in agv3"
rosrun gazebo_ros spawn_model -sdf -x 0.1 -y 0.1 -z 0.05 -R 0 -P 0 -Y 0 -file `rospack find nist_gear`/models/assembly_regulator_green_ariac/model.sdf -reference_frame agv3::kit_tray_3::kit_tray_3::tray -model assembly_regulator_green_10
rosrun gazebo_ros spawn_model -sdf -x -0.1 -y -0.1 -z 0.05 -R 0 -P 0 -Y 0.78 -file `rospack find nist_gear`/models/assembly_sensor_blue_ariac/model.sdf -reference_frame agv3::kit_tray_3::kit_tray_3::tray -model assembly_sensor_blue_10
echo "...Spawn products in agv4"
rosrun gazebo_ros spawn_model -sdf -x -0.15 -y -0.1 -z 0.05 -R 3.14 -P 0 -Y 0.78 -file `rospack find nist_gear`/models/assembly_pump_blue_ariac/model.sdf -reference_frame agv4::kit_tray_4::kit_tray_4::tray -model assembly_pump_blue_10
rosrun gazebo_ros spawn_model -sdf -x 0.1 -y 0.1 -z 0.05 -R 0 -P 0 -Y 0 -file `rospack find nist_gear`/models/assembly_battery_blue_ariac/model.sdf -reference_frame agv4::kit_tray_4::kit_tray_4::tray -model assembly_battery_blue_10
sleep 3
echo "...Submit AGV3"
rosservice call /ariac/agv3/submit_shipment "as3" "order_0_kitting_shipment_0"
sleep 5
echo "...Submit AGV4"
rosservice call /ariac/agv4/submit_shipment "as3" "order_1_kitting_shipment_0"
# sleep 3
# echo "...Submit kitting shipment"
# rosservice call /ariac/agv2/submit_shipment as1  "order_0_kitting_shipment_0"
# sleep 6
# echo "...Submit assembly"
# rosservice call /ariac/as1/submit_shipment "order_1_assembly_shipment_0"

