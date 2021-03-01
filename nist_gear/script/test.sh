#!/bin/bash

sleep 3
echo "...Start the competition"
rosservice call /ariac/start_competition
sleep 3
# echo "...Submit assembly"
# rosservice call /ariac/as2/submit_shipment "order_0_assembly_shipment_0"
# sleep 3
echo "...Submit kitting shipment"
rosservice call /ariac/agv2/submit_shipment as1  "order_0_kitting_shipment_0"
sleep 6
echo "...Submit assembly"
rosservice call /ariac/as1/submit_shipment "order_1_assembly_shipment_0"

