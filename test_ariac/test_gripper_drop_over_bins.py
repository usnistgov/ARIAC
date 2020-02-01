#!/usr/bin/env python

from __future__ import print_function

import sys
import time

from test_gripper import GripperTester
from ariac_example import ariac_example
import rospy
import rostest


class GripperBinDropTester(GripperTester):

    def test(self):
        self.comp_class = ariac_example.MyCompetitionClass()
        ariac_example.connect_callbacks(self.comp_class)
        time.sleep(1.0)

        self._send_arms_to_initial_pose()

        self._send_arm1_to_product()

        self._enable_gripper(arm=1)
        time.sleep(2.0)
        self.assertTrue(self.comp_class.arm_1_current_gripper_state.enabled)
        self.assertTrue(self.comp_class.arm_1_current_gripper_state.attached)

        self._send_arm1_to_tray()
        self.assertTrue(self.comp_class.arm_1_current_gripper_state.enabled)
        self.assertFalse(self.comp_class.arm_1_current_gripper_state.attached)


if __name__ == '__main__':
    rospy.init_node('test_gripper_bin_drop', anonymous=True)

    # Wait until /clock is being published; this can take an unpredictable
    # amount of time when we're downloading models.
    while rospy.Time.now().to_sec() == 0.0:
        print('Waiting for Gazebo to start...')
        time.sleep(1.0)
    # Take an extra nap, to allow plugins to be loaded
    time.sleep(12.0)
    print('OK, starting test.')

    rostest.run('test_ariac', 'test_gripper_bin_drop', GripperBinDropTester, sys.argv)
