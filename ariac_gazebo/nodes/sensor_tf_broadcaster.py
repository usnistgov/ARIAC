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
        sensors = data['sensors']
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

    # Send tf transforms
    sensor_tf_broadcaster.send_transforms()

    try:
        rclpy.spin(sensor_tf_broadcaster)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()


if __name__ == '__main__':
    main()