#!/usr/bin/env python

import rospy
from std_srvs.srv import Trigger
from nist_gear.msg import Orders
from test_competitor.assembly_commander import AssemblyPart
from nist_gear.srv import AssemblyStationSubmitShipment
from nist_gear.srv import SubmitKittingShipment


class Competitor:
    def __init__(self):
        self.received_order = False
        self.orders = []
        self.order_sub = rospy.Subscriber(
            "/ariac/orders", Orders, self.order_callback)

    def start_competition(self):
        rospy.loginfo("Waiting for competition to be ready...")
        rospy.wait_for_service('/ariac/start_competition')
        rospy.loginfo("Competition is now ready.")
        rospy.loginfo("Requesting competition start...")

        try:
            start = rospy.ServiceProxy('/ariac/start_competition', Trigger)
            response = start()
        except rospy.ServiceException as exc:
            rospy.logerr("Failed to start the competition: %s" % exc)
        if not response.success:
            rospy.logerr("Failed to start the competition: %s" % response)
        else:
            rospy.loginfo("Competition started!")

    def stop_competition(self):
        try:
            end = rospy.ServiceProxy('/ariac/end_competition', Trigger)
            response = end()
        except rospy.ServiceException as exc:
            rospy.logerr("Failed to stop the competition: %s" % exc)
        if not response.success:
            rospy.logerr("Failed to stop the competition: %s" % response)
        else:
            rospy.loginfo("Competition ended")

    def process_assembly_shipment(self, assembly_shipment):
        # Eventually read from order but build dictionary to start
        parts = []
        name_str = ""
        for product in assembly_shipment.products:
            _, part_type, color = product.type.split("_")

            parts.append(AssemblyPart(part_type, color, product.pose))
            name_str += (color + "_" + part_type + ", ")

        rospy.loginfo("Received assembly order to deliver " + name_str +
                      "to assembly station: " + assembly_shipment.station_id)

        return parts

    def process_kitting_shipment(self, kitting_shipment):
        # Eventually read from order but build dictionary to start
        parts = []
        for product in kitting_shipment.products:
            parts.append(product)

        return parts

    def submit_kitting_shipment(self, agv, assembly_station, shipment_type):
        """ ROS service call to submit a kitting shipment
        Returns:
        bool: status of the service call
        """
        rospy.wait_for_service('/ariac/' + agv + '/submit_kitting_shipment')
        rospy.ServiceProxy('/ariac/' + agv + '/submit_kitting_shipment',
                           SubmitKittingShipment)(assembly_station, shipment_type)

    def submit_assembly_shipment(self, station_id, shipment_type):
        # rospy.logerr("Submitting shipment")
        srv = '/ariac/' + station_id + '/submit_assembly_shipment'
        rospy.loginfo("Submitting shipment {} at station {}".format(
            shipment_type, station_id))
        rospy.wait_for_service(srv)
        try:
            submit_shipment = rospy.ServiceProxy(
                srv, AssemblyStationSubmitShipment)
            result = submit_shipment(shipment_type)
            rospy.loginfo(result.inspection_result)
            return result.success
        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)
        return False
        # req = AssemblyStationSubmitShipmentRequest()
        # req.shipment_type = shipment_type

        # return submit_shipment(req)

    def order_callback(self, msg):
        self.orders.append(msg)
        self.received_order = True
