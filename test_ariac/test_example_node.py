#!/usr/bin/env python

from __future__ import print_function

import sys
import time
import unittest

from ariac_example import ariac_example
from osrf_gear.srv import SubmitTray
from std_msgs.msg import Float32
import rospy
import rostest


class ExampleNodeTester(unittest.TestCase):

    def comp_score_callback(self, msg):
        self.current_comp_score = msg.data

    def prepare_tester(self):
        self.comp_class = ariac_example.MyCompetitionClass()
        ariac_example.connect_callbacks(self.comp_class)

        self.current_comp_score = None
        self.comp_state_sub = rospy.Subscriber(
            "/ariac/current_score", Float32, self.comp_score_callback)

        # Pre-defined initial pose because sometimes the arm starts "droopy"
        self._send_arms_to_initial_pose()

    def test(self):
        print("Preparing tester")
        self.prepare_tester()
        print("sending arm to zero state")
        self._test_send_arm_to_zero_state()

        # Starting the competition will cause products from the order to be spawned on the tray
        print("starting the competition")
        self._test_start_comp()
        time.sleep(5.0)
        print("checking that an order is received")
        self._test_order_reception()

        # Submit the trays 
        print("asking agv to deliver a tray")
        self._test_agv1_control()
        time.sleep(5.0)

        # Check the score
        print("checking competition state")
        self._test_comp_end()

    def _test_start_comp(self):
        success = ariac_example.start_competition()
        self.assertTrue(success, 'Failed to start the competition')
        time.sleep(1.5)
        self.assertTrue(
            self.comp_class.current_comp_state == 'go', 'Competition not in "go" state')

    def _test_order_reception(self):
        self.assertEqual(len(self.comp_class.received_orders), 1)
        num_products_in_order = len(self.comp_class.received_orders[0].shipments[0].products)
        self.assertGreater(num_products_in_order, 0, 'No products in received order')

    def _send_arms_to_initial_pose(self):
        self.comp_class.send_arm1_to_state([0] * len(self.comp_class.arm_joint_names))
        self.comp_class.send_arm2_to_state([0] * len(self.comp_class.arm_joint_names))
        time.sleep(1.0)

    def _test_send_arm_to_zero_state(self):
        self.comp_class.send_arm1_to_state([0] * len(self.comp_class.arm_joint_names))
        self.comp_class.send_arm2_to_state([0] * len(self.comp_class.arm_joint_names))
        # This can be slow if there are a lot of models in the environment
        time.sleep(5.0)
        error = 0
        for position in self.comp_class.arm_1_current_joint_state.position:
            error += abs(position - 0.0)
        self.assertTrue(error < 0.5, 'Arm was not properly sent to zero state')
        error = 0
        for position in self.comp_class.arm_2_current_joint_state.position:
            error += abs(position - 0.0)
        self.assertTrue(error < 0.5, 'Arm was not properly sent to zero state')

    def _test_agv1_control(self, shipment_id='order_0_shipment_0'):
        success = ariac_example.control_agv(shipment_id, agv_num=1)
        self.assertTrue(success, 'Failed to control agv1')

    def _test_submit_shipment(self, shipment_type, agv_num):
        success = ariac_example.submit_shipment(shipment_type, agv_num)
        self.assertTrue(success, 'failed to submit shipment')

    def _test_comp_end(self):
        num_received_orders = len(self.comp_class.received_orders)
        num_shipments = len(self.comp_class.received_orders[0].shipments)
        if num_received_orders == 1 and num_shipments == 1:
            self.assertTrue(
                self.comp_class.current_comp_state == 'done', 'Competition not in "done" state')
        else:
            # If there were more shipments expected, the order won't be done
            self.assertTrue(
                self.comp_class.current_comp_state == 'go', 'Competition not in "go" state')


if __name__ == '__main__':
    rospy.init_node('test_example_node', anonymous=True)

    # Wait until /clock is being published; this can take an unpredictable
    # amount of time when we're downloading models.
    while rospy.Time.now().to_sec() == 0.0:
        print('Waiting for Gazebo to start...')
        time.sleep(1.0)
    # Take an extra nap, to allow plugins to be loaded
    time.sleep(20.0)
    print('OK, starting test.')

    rostest.run('osrf_gear', 'test_example_node', ExampleNodeTester, sys.argv)
