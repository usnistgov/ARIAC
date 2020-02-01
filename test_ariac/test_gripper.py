#!/usr/bin/env python

from __future__ import print_function

import sys
import time

from test_example_node import ExampleNodeTester
from ariac_example import ariac_example
import rospy
import rostest


class GripperTester(ExampleNodeTester):

    def test(self):
        self.comp_class = ariac_example.MyCompetitionClass()
        ariac_example.connect_callbacks(self.comp_class)
        time.sleep(1.0)

        # Pre-defined initial pose because sometimes the arms start "droopy"
        self._send_arms_to_initial_pose()

        # Pre-defined pose that puts the gripper in contact with a product.
        self._send_arm1_to_product()
        self._send_arm2_to_product()

        # Enable the gripper so that it picks up the product.
        self._test_enable_gripper()

        self._send_arm1_to_tray()
        self._send_arm2_to_tray()

        # Disable the gripper so that it drops the product.
        self._test_disable_gripper()

    def _test_enable_gripper(self):
        self.assertFalse(self.comp_class.arm_1_current_gripper_state.enabled)
        self.assertFalse(self.comp_class.arm_1_current_gripper_state.attached)
        self.assertFalse(self.comp_class.arm_2_current_gripper_state.enabled)
        self.assertFalse(self.comp_class.arm_2_current_gripper_state.attached)

        self.assertTrue(self._enable_gripper(arm=1))
        time.sleep(2.0)
        self.assertTrue(self.comp_class.arm_1_current_gripper_state.enabled)
        self.assertTrue(self.comp_class.arm_1_current_gripper_state.attached)

        self.assertTrue(self._enable_gripper(arm=2))
        time.sleep(2.0)
        self.assertTrue(self.comp_class.arm_2_current_gripper_state.enabled)
        self.assertTrue(self.comp_class.arm_2_current_gripper_state.attached)

    def _enable_gripper(self, arm):
        success = ariac_example.control_gripper(True, arm=arm)
        time.sleep(0.5)
        return success

    def _test_disable_gripper(self):
        self.assertTrue(self.comp_class.arm_1_current_gripper_state.enabled)
        self.assertTrue(self.comp_class.arm_1_current_gripper_state.attached)
        self.assertTrue(self.comp_class.arm_2_current_gripper_state.enabled)
        self.assertTrue(self.comp_class.arm_2_current_gripper_state.attached)

        self.assertTrue(self._disable_gripper(arm=1))
        time.sleep(2.0)
        self.assertFalse(self.comp_class.arm_1_current_gripper_state.enabled)
        self.assertFalse(self.comp_class.arm_1_current_gripper_state.attached)

        self.assertTrue(self._disable_gripper(arm=2))
        time.sleep(2.0)
        self.assertFalse(self.comp_class.arm_2_current_gripper_state.enabled)
        self.assertFalse(self.comp_class.arm_2_current_gripper_state.attached)

    def _disable_gripper(self, arm):
        success = ariac_example.control_gripper(False, arm=arm)
        time.sleep(0.5)
        return success

    def _send_arm1_to_product(self):
        trajectory = [
            [3.14, -1.57, 2.14, 3.24, -1.59, 0.126, 0.0],
            [3.45, -0.75, 2.14, 3.24, -1.59, 0.126, 0.0],
            [3.6, -0.538, 2.14, 3.24, -1.59, 0.126, 0.0]
        ]
        for positions in trajectory:
            self.comp_class.send_arm1_to_state(positions)
            time.sleep(1.5)

    def _send_arm2_to_product(self):
        trajectory = [
            [0.0, -2.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            [3.14, -2.0, 2.14, 3.27, -1.51, 0.0, 0.0],
            [2.76, -0.6, 1.88, 3.27, -1.51, 0.0, 0.24],
            [2.76, -0.46, 1.88, 3.27, -1.51, 0.0, 0.24],
        ]
        for positions in trajectory:
            self.comp_class.send_arm2_to_state(positions)
            time.sleep(1.5)

    def _send_arm1_to_tray(self):
        trajectory = [
            [3.14, -1.57, 2.14, 3.27, -1.51, 0.0, 0.0],
            [1.85, 0, -0.38, 1.57, -1.51, 0.0, 1.0],
            [1.507, 0, -0.38, 0.38, -1.51, 0.0, 1.0],
            [1.507, 0.38, -0.38, 1.55, 1.75, 0.127, 1.18]
        ]
        for positions in trajectory:
            self.comp_class.send_arm1_to_state(positions)
            time.sleep(1.0)

    def _send_arm2_to_tray(self):
        trajectory = [
            [2.76, -0.47, 1.88, 3.27, -1.51, 0.0, 0.24],
            [2.76, -1.13, 1.88, 3.27, -1.51, 0.0, 0.24],
            [4.27, -1.13, 1.88, 3.27, -1.51, 0.0, -1.04],
            [4.52, -0.50, 1.51, 3.64, -1.51, 0.0, -1.04],
        ]
        for positions in trajectory:
            self.comp_class.send_arm2_to_state(positions)
            time.sleep(1.0)


if __name__ == '__main__':
    rospy.init_node('test_gripper', anonymous=True)

    # Wait until /clock is being published; this can take an unpredictable
    # amount of time when we're downloading models.
    while rospy.Time.now().to_sec() == 0.0:
        print('Waiting for Gazebo to start...')
        time.sleep(1.0)
    # Take an extra nap, to allow plugins to be loaded
    time.sleep(12.0)
    print('OK, starting test.')

    rostest.run('test_ariac', 'test_gripper', GripperTester, sys.argv)
