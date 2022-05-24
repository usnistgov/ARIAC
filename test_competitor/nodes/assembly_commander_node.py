#!/usr/bin/env python

import rospy
from test_competitor.assembly_commander import AssemblyCommander
from test_competitor.competitor import Competitor

# This code works with the trial configurations:
# assembly_as1.yaml
# assembly_as2.yaml
# assembly_as3.yaml
# assembly_as4.yaml

if __name__ == "__main__":
    rospy.init_node("assembly_commander_node")

    competitor = Competitor()
    commander = AssemblyCommander()
    rospy.on_shutdown(commander.shutdown)

    competitor.start_competition()

    # Wait for order to be recieved
    r = rospy.Rate(10)
    while not competitor.received_order:
        r.sleep()

    for order in competitor.orders:
        rospy.loginfo("Number of shipment to complete: {0}".format(
            len(order.assembly_shipments)))
        for assembly_shipment in order.assembly_shipments:

            parts = competitor.process_assembly_shipment(assembly_shipment)

            briefcase = 'briefcase_' + assembly_shipment.station_id[-1]

            commander.move_arm_to_home_position()
            commander.move_gantry_to_station(assembly_shipment.station_id)

            for part in parts:
                # Assume part is located on the kit tray, get the pose from logical camera
                tray_pose = commander.get_pose_on_kit_tray(
                    assembly_shipment.station_id, part)

                if not tray_pose:
                    rospy.logerr("Desired part not on kit tray.")
                    exit()

                commander.move_arm_to_home_position()

                if not commander.move_gantry_to_agv(assembly_shipment.station_id):
                    exit()

                commander.pick_part(
                    assembly_shipment.station_id, tray_pose, part)

                commander.move_arm_to_home_position()

                commander.move_gantry_to_assembly_position(briefcase, part)

                success = commander.assemble_part(briefcase, part)

                if not success:
                    commander.move_arm_to_home_position()
                    rospy.logerr("Dropping part..")
                    commander.gripper_off()

            rospy.loginfo("Shipment Completed")
            commander.move_arm_to_home_position()

            rospy.loginfo(competitor.submit_assembly_shipment(
                assembly_shipment.station_id,
                assembly_shipment.shipment_type))
