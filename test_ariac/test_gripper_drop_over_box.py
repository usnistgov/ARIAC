#!/usr/bin/env python

from __future__ import print_function

import sys
import time

from test_gripper import GripperTester
from test_tf_frames import TfTester
from ariac_example import ariac_example
import rospy
import rostest

import tf
import tf2_geometry_msgs  # noqa
import tf2_py as tf2
import tf2_ros



class GripperBoxDropTester(GripperTester, TfTester):

    def test(self):
        self.comp_class = ariac_example.MyCompetitionClass()
        ariac_example.connect_callbacks(self.comp_class)
        time.sleep(1.0)
        self.prepare_tf()

        self._send_arms_to_initial_pose()

        self._send_arm1_to_product()

        self._enable_gripper(arm=1)
        time.sleep(2.0)
        self.assertTrue(self.comp_class.arm_1_current_gripper_state.enabled)
        self.assertTrue(self.comp_class.arm_1_current_gripper_state.attached)

        self._send_arm1_to_tray()
        self.assertTrue(self.comp_class.arm_1_current_gripper_state.enabled)
        self.assertFalse(self.comp_class.arm_1_current_gripper_state.attached)

        self._test_dropped_product_pose()

        time.sleep(1.0)

    def _test_dropped_product_pose(self):
        self._test_pose(
            [0.15, 0.15, 0.0],
            tf.transformations.quaternion_from_euler(0, 0, 0.2),
            self.camera_frame + '_gasket_part_5_frame',
            'kit_tray_1'
        )


if __name__ == '__main__':
    rospy.init_node('test_gripper_box_drop', anonymous=True)

    # Wait until /clock is being published; this can take an unpredictable
    # amount of time when we're downloading models.
    while rospy.Time.now().to_sec() == 0.0:
        print('Waiting for Gazebo to start...')
        time.sleep(1.0)
    # Take an extra nap, to allow plugins to be loaded
    time.sleep(12.0)
    print('OK, starting test.')

    rostest.run('test_ariac', 'test_gripper_box_drop', GripperBoxDropTester, sys.argv)
