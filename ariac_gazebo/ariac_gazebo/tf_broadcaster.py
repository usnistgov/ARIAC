#!/usr/bin/env python3

from rclpy.node import Node

from geometry_msgs.msg import TransformStamped
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

class TFBroadcaster(Node):
    def __init__(self, name):
        super().__init__(name)
        self.tf_static_broadcaster = StaticTransformBroadcaster(self)
        self.transforms = []
    
    def generate_transform(self, parent_frame, child_frame, initial_pose):
        t = TransformStamped()

        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = parent_frame
        t.child_frame_id = child_frame

        t.transform.translation.x = initial_pose.position.x
        t.transform.translation.y = initial_pose.position.y
        t.transform.translation.z = initial_pose.position.z
        t.transform.rotation.x = initial_pose.orientation.x
        t.transform.rotation.y = initial_pose.orientation.y
        t.transform.rotation.z = initial_pose.orientation.z
        t.transform.rotation.w = initial_pose.orientation.w
        
        self.transforms.append(t)
    
    def send_transforms(self):
        self.tf_static_broadcaster.sendTransform(self.transforms)