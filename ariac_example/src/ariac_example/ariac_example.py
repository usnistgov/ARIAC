#!/usr/bin/env python
# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from __future__ import print_function

import time

import rospy

from nist_gear.msg import Order
from nist_gear.msg import VacuumGripperState
from nist_gear.srv import AGVControl
from nist_gear.srv import ConveyorBeltControl
from nist_gear.srv import DroneControl
from nist_gear.srv import SubmitShipment
from nist_gear.srv import VacuumGripperControl
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint


def start_competition():
    rospy.loginfo("Waiting for competition to be ready...")
    rospy.wait_for_service('/ariac/start_competition')
    rospy.loginfo("Competition is now ready.")
    rospy.loginfo("Requesting competition start...")

    try:
        start = rospy.ServiceProxy('/ariac/start_competition', Trigger)
        response = start()
    except rospy.ServiceException as exc:
        rospy.logerr("Failed to start the competition: %s" % exc)
        return False
    if not response.success:
        rospy.logerr("Failed to start the competition: %s" % response)
    else:
        rospy.loginfo("Competition started!")
    return response.success


def control_gripper(enabled, arm):
    if arm not in (1, 2):
        raise ValueError('Only two arms (1 or 2)')
    rospy.loginfo("Waiting for gripper control to be ready...")
    service_name = '/ariac/arm{}/gripper/control'.format(arm)
    rospy.wait_for_service(service_name)
    rospy.loginfo("Gripper control is now ready.")
    rospy.loginfo("Requesting gripper control...")

    try:
        gripper_control = rospy.ServiceProxy(service_name, VacuumGripperControl)
        response = gripper_control(enabled)
    except rospy.ServiceException as exc:
        rospy.logerr("Failed to control the gripper: %s" % exc)
        return False
    if not response.success:
        rospy.logerr("Failed to control the gripper: %s" % response)
    else:
        rospy.loginfo("Gripper controlled successfully")
    return response.success


class MyCompetitionClass:
    def __init__(self):
        self.arm_1_joint_trajectory_publisher = \
            rospy.Publisher("/ariac/arm1/arm/command", JointTrajectory, queue_size=10)
        self.arm_2_joint_trajectory_publisher = \
            rospy.Publisher("/ariac/arm2/arm/command", JointTrajectory, queue_size=10)
        self.current_comp_state = None
        self.received_orders = []
        self.arm_1_current_joint_state = None
        self.arm_2_current_joint_state = None
        self.arm_1_current_gripper_state = None
        self.arm_2_current_gripper_state = None
        self.last_arm_1_joint_state_print = time.time()
        self.last_arm_2_joint_state_print = time.time()
        self.last_arm_1_gripper_state_print = time.time()
        self.last_arm_2_gripper_state_print = time.time()
        self.arm_1_has_been_zeroed = False
        self.arm_2_has_been_zeroed = False
        self.arm_joint_names = [
            'shoulder_pan_joint',
            'shoulder_lift_joint',
            'elbow_joint',
            'wrist_1_joint',
            'wrist_2_joint',
            'wrist_3_joint',
            'linear_arm_actuator_joint'
        ]

    def comp_state_callback(self, msg):
        if self.current_comp_state != msg.data:
            rospy.loginfo("Competition state: " + str(msg.data))
        self.current_comp_state = msg.data

    def order_callback(self, msg):
        rospy.loginfo("Received order:\n" + str(msg))
        self.received_orders.append(msg)

    def arm_1_joint_state_callback(self, msg):
        if time.time() - self.last_arm_1_joint_state_print >= 10:
            rospy.loginfo("Current Arm 1 Joint States (throttled to 0.1 Hz):\n" + str(msg))
            self.last_arm_1_joint_state_print = time.time()
        self.arm_1_current_joint_state = msg

    def arm_2_joint_state_callback(self, msg):
        if time.time() - self.last_arm_2_joint_state_print >= 10:
            rospy.loginfo("Current Arm 2 Joint States (throttled to 0.1 Hz):\n" + str(msg))
            self.last_arm_2_joint_state_print = time.time()
        self.arm_2_current_joint_state = msg

    def arm_1_gripper_state_callback(self, msg):
        if time.time() - self.last_arm_1_gripper_state_print >= 10:
            rospy.loginfo("Current Arm 1 gripper state (throttled to 0.1 Hz):\n" + str(msg))
            self.last_arm_1_gripper_state_print = time.time()
        self.arm_1_current_gripper_state = msg

    def arm_2_gripper_state_callback(self, msg):
        if time.time() - self.last_arm_2_gripper_state_print >= 10:
            rospy.loginfo("Current Arm 2 gripper state (throttled to 0.1 Hz):\n" + str(msg))
            self.last_arm_2_gripper_state_print = time.time()
        self.arm_2_current_gripper_state = msg

    def send_arm_to_state(self, positions, publisher):
        msg = JointTrajectory()
        msg.joint_names = self.arm_joint_names
        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start = rospy.Duration(1.0)
        msg.points = [point]
        rospy.loginfo("Sending command:\n" + str(msg))
        publisher.publish(msg)

    def send_arm1_to_state(self, positions):
        return self.send_arm_to_state(positions, self.arm_1_joint_trajectory_publisher)

    def send_arm2_to_state(self, positions):
        return self.send_arm_to_state(positions, self.arm_2_joint_trajectory_publisher)


def connect_callbacks(comp_class):
    comp_state_sub = rospy.Subscriber(
        "/ariac/competition_state", String, comp_class.comp_state_callback)
    order_sub = rospy.Subscriber("/ariac/orders", Order, comp_class.order_callback)
    joint_state_sub = rospy.Subscriber(
        "/ariac/arm1/joint_states", JointState, comp_class.arm_1_joint_state_callback)
    joint_state_sub = rospy.Subscriber(
        "/ariac/arm2/joint_states", JointState, comp_class.arm_2_joint_state_callback)
    gripper1_state_sub = rospy.Subscriber(
        "/ariac/arm1/gripper/state", VacuumGripperState, comp_class.arm_1_gripper_state_callback)
    gripper2_state_sub = rospy.Subscriber(
        "/ariac/arm2/gripper/state", VacuumGripperState, comp_class.arm_2_gripper_state_callback)


def control_agv(shipment_type, agv_num):
    if agv_num not in (1,2):
        raise ValueError('agv_num must be 1 or 2')

    rospy.loginfo("Waiting for agv{} control to be ready...".format(agv_num))
    name = '/ariac/agv{}'.format(agv_num)
    rospy.wait_for_service(name)
    rospy.loginfo("agv{} control is now ready.".format(agv_num))
    rospy.loginfo("Requesting agv control...")

    try:
        agv_control = rospy.ServiceProxy(name, AGVControl)
        response = agv_control(shipment_type)
    except rospy.ServiceException as exc:
        rospy.logerr("Failed to control the agv: %s" % exc)
        return False
    if not response.success:
        rospy.logerr("Failed to control the agv: %s" % response)
    else:
        rospy.loginfo("agv controlled successfully")
    return response.success


def submit_shipment(shipment_type, agv_num):
    if agv_num not in (1,2):
        raise ValueError('agv_num must be 1 or 2')

    rospy.loginfo("Waiting for submit shipment to be ready...".format(agv_num))
    name = '/ariac/submit_shipment'
    rospy.wait_for_service(name)
    rospy.loginfo("submit_shipment is now ready.")
    rospy.loginfo("Requesting shipment")

    try:
        submit_shipment = rospy.ServiceProxy(name, SubmitShipment)
        response = submit_shipment(destination_id=str(agv_num), shipment_type=shipment_type)
    except rospy.ServiceException as exc:
        rospy.logerr("Failed to submit shipment: %s" % exc)
        return False
    if not response.success:
        rospy.logerr("Failed to submit shipment: %s" % response)
    else:
        rospy.loginfo("shipment submitted successfully")
    return response.success
