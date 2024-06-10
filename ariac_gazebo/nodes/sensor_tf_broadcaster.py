#!/usr/bin/env python3

import sys
import yaml

import rclpy

from ament_index_python.packages import get_package_share_directory

from ariac_gazebo.tf_broadcaster import TFBroadcaster
from ariac_gazebo.utilities import pose_info

def main():
    rclpy.init()

    sensor_tf_broadcaster = TFBroadcaster("sensor_tf_broadcaster")

    config = sys.argv[1]

    with open(config, "r") as stream:
        try:
            data = yaml.safe_load(stream)
        except yaml.YAMLError as exc:
            print(exc)
    
    try:
        sensors = data['static_sensors']
    except (TypeError, KeyError):
        sensor_tf_broadcaster.get_logger().warn("No sensors found in config")
        sensors = []

    if not sensors:
        sensors = []

    for sensor_name in sensors:
        xyz = sensors[sensor_name]['pose']['xyz']
        rpy = sensors[sensor_name]['pose']['rpy']

        pose = pose_info(xyz, rpy)

        sensor_tf_broadcaster.generate_transform('world', sensor_name + "_frame", pose)

        # Publish optical frame for cameras
        try:
            if 'rgb' in sensors[sensor_name]['type']:
                xyz = [0, 0, 0]
                rpy = ['-pi/2', 0, '-pi/2']
                
                optical_pose = pose_info(xyz, rpy)

                sensor_tf_broadcaster.generate_transform(
                    sensor_name + "_frame", sensor_name + "_optical_frame", optical_pose)

        except KeyError:
            pass

    # Send tf transforms
    sensor_tf_broadcaster.send_transforms()

    try:
        rclpy.spin(sensor_tf_broadcaster)
    except KeyboardInterrupt:
        sensor_tf_broadcaster.destroy_node()


if __name__ == '__main__':
    main()