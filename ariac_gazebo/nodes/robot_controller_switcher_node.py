#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from ariac_msgs.msg import Robots
from controller_manager_msgs.srv import SwitchController

from rclpy.executors import SingleThreadedExecutor

class RobotControllerSwitcher(Node):
    def __init__(self):
        super().__init__('robot_controller_switcher')
        
        # Create service client to spawn objects into gazebo
        self.controller_switcher = self.create_client(SwitchController, '/controller_manager/switch_controller')
        self.request = SwitchController.Request()
        self.request.strictness = SwitchController.Request.BEST_EFFORT

        self.floor_robot_controllers = ["floor_robot_controller", "linear_rail_controller"]
        self.ceiling_robot_controllers = ["ceiling_robot_controller", "gantry_controller"]

        self.recieved_msg = False
        self.robot_health_sub = self.create_subscription(Robots, 'ariac/robot_health', self.robot_health_cb, 10)

        self.ceiling_robot_state = False
        self.floor_robot_state = False
        
        self.ceiling_robot_health = False
        self.floor_robot_health = False
    
    def run(self):
        while rclpy.ok():
            try:
                rclpy.spin_once(self)
            except KeyboardInterrupt:
                break

            if self.recieved_msg:
                self.request.start_controllers.clear()
                self.request.stop_controllers.clear()

                if self.floor_robot_health and not self.floor_robot_state:
                    for controller in self.floor_robot_controllers:
                        self.request.start_controllers.append(controller)
                        self.floor_robot_state = True
                
                if self.ceiling_robot_health and not self.ceiling_robot_state:
                    for controller in self.ceiling_robot_controllers:
                        self.request.start_controllers.append(controller)
                        self.ceiling_robot_state = True
                
                if not self.floor_robot_health and self.floor_robot_state:
                    for controller in self.floor_robot_controllers:
                        self.request.stop_controllers.append(controller)
                        self.floor_robot_state = False

                if not self.ceiling_robot_health and self.ceiling_robot_state:
                    for controller in self.ceiling_robot_controllers:
                        self.request.stop_controllers.append(controller)
                        self.ceiling_robot_state = False

                if self.request.start_controllers or self.request.stop_controllers:
                    future = self.controller_switcher.call_async(self.request)

                    rclpy.spin_until_future_complete(self, future)

                    if not future.result().ok:
                        self.get_logger().error("Could not switch controllers")
                
                self.recieved_msg = False

                    
    def robot_health_cb(self, msg: Robots):
        # self.get_logger().info("Recieved msg")
        self.recieved_msg = True
        self.ceiling_robot_health = msg.ceiling_robot
        self.floor_robot_health = msg.floor_robot


if __name__ == "__main__":
    rclpy.init()

    robot_controller_switcher = RobotControllerSwitcher()

    robot_controller_switcher.run()

    rclpy.shutdown()

    