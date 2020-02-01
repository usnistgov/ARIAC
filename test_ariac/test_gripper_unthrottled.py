#!/usr/bin/env python

from __future__ import print_function

import sys
import time

from test_gripper import GripperTester
from ariac_example import ariac_example
import rospy
import rostest


class UnthrottledGripperTester(GripperTester):

    def test(self):
        self.comp_class = ariac_example.MyCompetitionClass()
        ariac_example.connect_callbacks(self.comp_class)
        time.sleep(1.0)

        # Pre-defined initial pose because sometimes the arm starts "droopy"
        self._send_arm_to_initial_pose()

        # Pre-defined pose that puts the gripper in contact with a product.
        self._send_arm_to_product()

        # Pick up and drop the product multiple times.
        # Regression test for https://bitbucket.org/osrf/ariac/issues/61
        for i in range(50):
            print('Enabling & disabling gripper for the {0}th time...'.format(i))
            self._enable_gripper()
            self._disable_gripper()

        # Enable the gripper so that it picks up the product.
        self._test_enable_gripper()

        # Move the product over the shipping box using a pre-defined sequence of poses.
        self._send_arm_to_shipping_box()
        self.assertTrue(
            self.comp_class.current_gripper_state.enabled, 'Gripper no longer enabled')
        self.assertTrue(
            self.comp_class.current_gripper_state.attached, 'Product no longer attached')

        # Disable the gripper so that it drops the product.
        self._test_disable_gripper()

        time.sleep(1.0)


if __name__ == '__main__':
    rospy.init_node('test_gripper_unthrottled', anonymous=True)

    # Wait until /clock is being published; this can take an unpredictable
    # amount of time when we're downloading models.
    while rospy.Time.now().to_sec() == 0.0:
        print('Waiting for Gazebo to start...')
        time.sleep(1.0)
    # Take an extra nap, to allow plugins to be loaded
    time.sleep(12.0)
    print('OK, starting test.')

    rostest.run('test_ariac', 'test_gripper_unthrottled', UnthrottledGripperTester, sys.argv)
