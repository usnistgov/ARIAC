#!/usr/bin/env python

from __future__ import print_function

import math
import sys
import time

import geometry_msgs.msg
import rospy
import rostest
from test_example_node import ExampleNodeTester

import tf
import tf2_geometry_msgs  # noqa
import tf2_py as tf2
import tf2_ros


class TfTester(ExampleNodeTester):

    def test(self):
        self.prepare_tester()

        self.prepare_tf()

        self._test_tray_pose()
        self._test_logical_camera_products()
        self._test_faulty_products()

    def prepare_tf(self):
        self.camera_frame = 'logical_camera_above_tray_1'
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)

    def _test_tray_pose(self):
        self._test_pose(
            [0.0, 6.3, 0.0],
            tf.transformations.quaternion_from_euler(0, 0, math.pi),
            'kit_tray_1',
            'kit_tray_2',
        )

    def _test_logical_camera_products(self):
        self._test_pose(
            [0.1, -0.2, 0.0],
            tf.transformations.quaternion_from_euler(0, 0, 0),
            self.camera_frame + '_piston_rod_part_1_frame',
            'kit_tray_1'
        )

    def _test_faulty_products(self):
        quality_control_sensor = 'quality_control_sensor_1'
        # This product is faulty and should be reported as such in an anonymized way.
        self._test_pose(
            [0.1, -0.2, 0.0], tf.transformations.quaternion_from_euler(0, 0, 0),
            quality_control_sensor + '_model_1_frame',
            'kit_tray_1'
        )

        # The model type should not be used in the name anymore (now anonymized).
        with self.assertRaises(tf2.LookupException):
            self._test_pose(
                [0.1, -0.2, 0.0], tf.transformations.quaternion_from_euler(0, 0, 0),
                quality_control_sensor + '_piston_rod_part_1_frame',
                'kit_tray_1'
            )

        # This product is not faulty and should not be found by TF.
        with self.assertRaises(tf2.LookupException):
            self._test_pose(
                [0.1, -0.2, 0.0], tf.transformations.quaternion_from_euler(0, 0, 0),
                quality_control_sensor + '_model_3_frame',
                'kit_tray_1'
            )

    def _test_pose(self, position, orientation, frame_id, parent_frame_id='world'):
        expected_pose = geometry_msgs.msg.PoseStamped()
        expected_pose.pose.position.x = position[0]
        expected_pose.pose.position.y = position[1]
        expected_pose.pose.position.z = position[2]
        expected_pose.pose.orientation.x = orientation[0]
        expected_pose.pose.orientation.y = orientation[1]
        expected_pose.pose.orientation.z = orientation[2]
        expected_pose.pose.orientation.w = orientation[3]
        expected_pose.header.frame_id = parent_frame_id

        # Ensure that the transform is available.
        self.tfBuffer.lookup_transform(
            expected_pose.header.frame_id, frame_id, rospy.Time(), rospy.Duration(1.0)
        )

        # Transform the origin from one frame to the other.
        local_pose = geometry_msgs.msg.PoseStamped()
        local_pose.pose.orientation.w = 1
        local_pose.header.frame_id = frame_id
        world_pose = self.tfBuffer.transform(local_pose, expected_pose.header.frame_id)
        # The returned pose doesn't have its quaternion normalized...
        q = world_pose.pose.orientation
        length = (q.x**2 + q.y**2 + q.z**2 + q.w**2)**(.5)
        q.x /= length
        q.y /= length
        q.z /= length
        q.w /= length

        rospy.loginfo(
            'Checking pose of "' + frame_id + '" in frame "' + expected_pose.header.frame_id + '"')
        rospy.loginfo('Expected:')
        rospy.loginfo(str(expected_pose))
        rospy.loginfo('Calculated:')
        rospy.loginfo(str(world_pose))

        # Check that the transformed pose is as expected.
        tol = 0.05
        self.assertEqual(world_pose.header.frame_id, expected_pose.header.frame_id)
        self.assertAlmostEqual(world_pose.pose.position.x, expected_pose.pose.position.x, delta=tol)
        self.assertAlmostEqual(world_pose.pose.position.y, expected_pose.pose.position.y, delta=tol)
        self.assertAlmostEqual(world_pose.pose.position.z, expected_pose.pose.position.z, delta=tol)
        self.assertAlmostEqual(world_pose.pose.orientation.x, expected_pose.pose.orientation.x, delta=tol)
        self.assertAlmostEqual(world_pose.pose.orientation.y, expected_pose.pose.orientation.y, delta=tol)
        self.assertAlmostEqual(world_pose.pose.orientation.z, expected_pose.pose.orientation.z, delta=tol)
        self.assertAlmostEqual(world_pose.pose.orientation.w, expected_pose.pose.orientation.w, delta=tol)


if __name__ == '__main__':
    rospy.init_node('test_tf_frames', anonymous=True)

    # Wait until /clock is being published; this can take an unpredictable
    # amount of time when we're downloading models.
    while rospy.Time.now().to_sec() == 0.0:
        print('Waiting for Gazebo to start...')
        time.sleep(1.0)
    # Take an extra nap, to allow plugins to be loaded
    time.sleep(10.0)
    print('OK, starting test.')

    rostest.run('test_ariac', 'test_tf_frames', TfTester, sys.argv)
