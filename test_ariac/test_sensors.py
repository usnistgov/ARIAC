#!/usr/bin/env python

from __future__ import print_function

import sys
import time

import rospy
import rostest
from test_example_node import ExampleNodeTester

from nist_gear.msg import LogicalCameraImage
from nist_gear.msg import Proximity
from sensor_msgs.msg import Range
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import PointCloud


class SensorsTester(ExampleNodeTester):

    def test(self):
        self.prepare_tester()

        # Starting the competition will cause the sensor blackout period to be triggered.
        self._test_start_comp()

        # Subscribe to sensors.
        self.subscribe_to_sensors()
        start_time = rospy.Time.now()
	while (rospy.Time.now() - start_time).to_sec() < 5:
            rospy.sleep(0.1)
        self._test_messages_received()

    def add_sensor_callback(self, sensor_name):
        def sensor_callback(msg):
            self.callbacks_received.add(sensor_name)

        setattr(self, 'sensor_callback_' + sensor_name, sensor_callback)
        return sensor_callback

    def _test_messages_received(self):
        for sensor_name in self.sensors.keys():
            self.assertTrue(
                sensor_name in self.callbacks_received,
                'Callback not received from sensor: ' + sensor_name)

    def subscribe_to_sensors(self):
        self.sensors = {
            'logical_camera_1': LogicalCameraImage,
            'quality_control_sensor_1': LogicalCameraImage,
            'proximity_sensor_1': Range,
            'depth_camera_1': PointCloud,
            'laser_profiler_1': LaserScan,
            'break_beam_1': Proximity,
        }
        self.callbacks_received = set()
        for sensor_name, message_type in self.sensors.items():
            topic_name = '/ariac/' + sensor_name
            sensor_callback = self.add_sensor_callback(sensor_name)
            rospy.Subscriber(topic_name, message_type, sensor_callback)


if __name__ == '__main__':
    rospy.init_node('test_sensors', anonymous=True)

    # Wait until /clock is being published; this can take an unpredictable
    # amount of time when we're downloading models.
    while rospy.Time.now().to_sec() == 0.0:
        print('Waiting for Gazebo to start...')
        time.sleep(1.0)
    # Take an extra nap, to allow plugins to be loaded
    time.sleep(10.0)
    print('OK, starting test.')

    rostest.run('test_ariac', 'test_sensors', SensorsTester, sys.argv)
