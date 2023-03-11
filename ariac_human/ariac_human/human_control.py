#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool
from geometry_msgs.msg import Point, PoseStamped, Vector3, TransformStamped
from ariac_msgs.msg import HumanState

from std_srvs.srv import Trigger

from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros import TransformException

from ariac_gazebo.utilities import quaternion_from_euler

from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

class HumanControl(Node):
    def __init__(self):
        super().__init__("human_control_node")
        
        # TF
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Teleport Service Client
        self.teleport_human_client = self.create_client(Trigger, '/ariac_human/teleport')
        self.teleport_request = Trigger.Request()
    
        # Safe_zone_penalty Service Client
        self.s_zone_penalty_client = self.create_client(Trigger, '/ariac/set_human_safe_zone_penalty')
        self.safe_zone_penalty_request = Trigger.Request()
    
        # Subscribers
        self.stop_sub = self.create_subscription(Bool, '/ariac_human/stop', self.stop_human, 10)
        self.go_home_sub = self.create_subscription(Bool, '/ariac_human/go_home', self.go_home, 10)
        self.send_to_goal_sub = self.create_subscription(Point, '/ariac_human/goal_position', self.send_to_goal, 10)

        # Timer
        self.first_pub = True
        self.timer_rate = 0.1
        self.update_timer = self.create_timer(self.timer_rate, self.on_update)

        # Publisher: Human State
        self.state = HumanState()
        self.state_pub = self.create_publisher(HumanState, '/ariac_human/state', 10)

        # Publisher: End-Of-Movement
        self.eom_pub = self.create_publisher(topic='/ariac_human/position_reached', 
                                             msg_type=Bool, qos_profile = 10)

        # Publisher: Unsafe distance (TRUE requires stopping the robot and teleporting the human)
        self.unsafe_distance_pub = self.create_publisher(topic='/ariac_human/unsafe_distance', 
                                                         msg_type=Bool, qos_profile = 10)

        # Initial Pose
        self.initial_pose = PoseStamped()
        self.initial_pose.header.frame_id = "map"     
        self.initial_pose.pose.position.x = -15.0
        self.initial_pose.pose.position.y = -10.0
        self.initial_pose.pose.position.z = 0.0

        q = quaternion_from_euler(0, 0, 3.14)
        self.initial_pose.pose.orientation.x = q[0]
        self.initial_pose.pose.orientation.y = q[1]
        self.initial_pose.pose.orientation.z = q[2]
        self.initial_pose.pose.orientation.w = q[3]

        # Nav2 Basic Navigator
        self.has_active_goal = False
        self.goal_pose = PoseStamped()
        self.goal_pose.header.frame_id = "map"

        q = quaternion_from_euler(0, 0, 0)
        self.goal_pose.pose.orientation.x = q[0]
        self.goal_pose.pose.orientation.y = q[1]
        self.goal_pose.pose.orientation.z = q[2]
        self.goal_pose.pose.orientation.w = q[3]

        self.navigator = BasicNavigator()
        self.navigator.waitUntilNav2Active()
        self.navigator.setInitialPose(self.initial_pose)
        
    def stop_human(self, msg: Bool):
        if msg.data:
            self.navigator.cancelTask()
            self.has_active_goal = False

    def go_home(self, msg: Bool):
        if msg.data:
            self.navigator.cancelTask()
            self.navigator.setInitialPose(self.initial_pose)

            self.has_active_goal = False
            self.teleport_human_client.call_async(self.teleport_request)
            self.s_zone_penalty_client.call_async(self.safe_zone_penalty_request)

    def send_to_goal(self, msg: Point):
        self.has_active_goal = True

        self.goal_pose.pose.position.x = msg.x
        self.goal_pose.pose.position.y = msg.y
        self.goal_pose.pose.position.z = msg.z
        
        self.navigator.goToPose(self.goal_pose)

    def on_update(self):
        # Lookup transforms for human and gantry
        try:
            human_t = self.tf_buffer.lookup_transform('world', 'base_link', rclpy.time.Time())
            robot_t = self.tf_buffer.lookup_transform('world', 'torso_base', rclpy.time.Time())
        except TransformException:
            return
  
        human_position = self.fill_point_from_transform(human_t)
        robot_position = self.fill_point_from_transform(robot_t)

        # Calculate instantaneous velocity   
        if self.first_pub:
            self.first_pub = False
        else:
            if not human_t.header.stamp.nanosec == self.human_last_stamp:
                self.state.human_velocity = self.calculate_velocity(
                    human_position, 
                    self.state.human_position, 
                    human_t.header.stamp.nanosec - self.human_last_stamp)

            if not robot_t.header.stamp.nanosec == self.robot_last_stamp:
                self.state.robot_velocity = self.calculate_velocity(
                    robot_position, 
                    self.state.robot_position,
                    robot_t.header.stamp.nanosec - self.robot_last_stamp)        

        self.human_last_stamp = human_t.header.stamp.nanosec
        self.robot_last_stamp = robot_t.header.stamp.nanosec

        self.state.human_position = human_position
        self.state.robot_position = robot_position

        # Check Safe Distance
        if not self.check_safe_distance():
            msg = Bool()
            msg.data = True
            self.unsafe_distance_pub.publish(msg)

        if self.has_active_goal and self.navigator.isTaskComplete():
            self.has_active_goal = False
            msg = Bool()
            msg.data = True
            self.eom_pub.publish(msg)

        # Publish State
        self.state_pub.publish(self.state)
        
    def check_safe_distance(self) -> bool: 
        t_1 = 1
        t_2 = 1.5
        beta = 0
        delta = 1

        speed_h = math.sqrt(self.state.human_velocity.x**2 + self.state.human_velocity.y**2)
        speed_r = math.sqrt(self.state.robot_velocity.x**2 + self.state.robot_velocity.y**2)

        safe_distance = (speed_h * (t_1 + t_2)) + (speed_r * (t_1)) + beta + delta

        actual_distance = math.sqrt((self.state.human_position.x - self.state.robot_position.x)**2 + 
                                    (self.state.human_position.y - self.state.robot_position.y)**2)

        if actual_distance < safe_distance:
            return False
        else:
            return True

    def calculate_velocity(self, p1: Point, p2: Point, dt_nano: float) -> Vector3:
        dt = dt_nano / 1E9
        
        vel = Vector3()
        vel.x = (p1.x - p2.x)/dt
        vel.y = (p1.y - p2.y)/dt
        vel.z = (p1.z - p2.z)/dt

        return vel

    def fill_point_from_transform(self, t: TransformStamped) -> Point:
        point = Point()
        
        point.x = t.transform.translation.x
        point.y = t.transform.translation.y
        point.z = t.transform.translation.z

        return point