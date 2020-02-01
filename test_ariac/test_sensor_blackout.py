#!/usr/bin/env python

from __future__ import print_function

import sys
import time

import geometry_msgs.msg
import rospy
import rostest
from test_sensors import SensorsTester
from test_tf_frames import TfTester

import tf
import tf2_geometry_msgs  # noqa
import tf2_py as tf2
import tf2_ros

class SensorBlackoutTester(TfTester, SensorsTester):

    def test(self):
        self.prepare_tester()

        # Starting the competition will cause the sensor blackout period to be triggered.
        self._test_start_comp()

        # Wait for the sensor blackout to be triggered.
        rospy.sleep(2.0)

        # The TF frames from the quality control sensor should not be published.
        self.prepare_tf()
        self._test_no_tf_frames()

        # Subscribe to sensors.
        # No messages should be received during the blackout period.
        # This requires a change to the default behaviour of gazebo_plugins to not re-enable the
        # sensors when subscribers connect.
        self.subscribe_to_sensors()
        start_time = rospy.Time.now()
	while (rospy.Time.now() - start_time).to_sec() < 5:
            rospy.sleep(0.1)
        self._test_no_messages_received()

    def _test_no_messages_received(self):
        for sensor_name in self.sensors.keys():
            self.assertTrue(
                sensor_name not in self.callbacks_received,
                'Callback received from sensor: ' + sensor_name)

    def _test_no_tf_frames(self):
        with self.assertRaises(tf2.LookupException):
            self._test_faulty_products()


if __name__ == '__main__':
    rospy.init_node('test_sensor_blackout', anonymous=True)

    # Wait until /clock is being published; this can take an unpredictable
    # amount of time when we're downloading models.
    while rospy.Time.now().to_sec() == 0.0:
        print('Waiting for Gazebo to start...')
        time.sleep(1.0)
    # Take an extra nap, to allow plugins to be loaded
    time.sleep(10.0)
    print('OK, starting test.')

    rostest.run('test_ariac', 'test_sensor_blackout', SensorBlackoutTester, sys.argv)
