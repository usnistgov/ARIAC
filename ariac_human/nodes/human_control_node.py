#!/usr/bin/env python3

import rclpy

from ariac_human.human_control import HumanControl

if __name__ == "__main__":
    rclpy.init()

    human_control_node = HumanControl()

    try:
        rclpy.spin(human_control_node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()
