#!/usr/bin/env python3

import sys
import rclpy
import time

from ariac_gazebo.environment_startup import EnvironmentStartup

def main():
    rclpy.init()

    startup_node = EnvironmentStartup()

    # Wait five seconds for gazebo to start up
    time.sleep(2)

    # startup_node.pause_physics()

    # Spawn robots
    startup_node.spawn_robots()

    # Spawn sensors
    startup_node.spawn_sensors()

    # Spawn parts in bins
    startup_node.spawn_bin_parts()

    # Spawn kit trays
    startup_node.spawn_kit_trays()

    # Spawn trays and parts on AGVs
    startup_node.spawn_parts_on_agvs()

    # Read conveyor part config
    startup_node.parse_conveyor_config()
    
    # Parse the trial config file
    startup_node.parse_trial_file()

    # startup_node.unpause_physics()

    try:
        rclpy.spin(startup_node)
    except:
        startup_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()