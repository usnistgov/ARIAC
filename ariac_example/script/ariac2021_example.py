#!/usr/bin/env python

"""
Author: Pavel Piliptchak
Contact: pavel.piliptchak@nist.gov

This software was developed by employees of the National Institute of
Standards and Technology (NIST), an agency of the Federal Government and is
being made available as a public service. Pursuant to title 17
United States Code Section 105, works of NIST employees are not subject to
copyright protection in the United States.  This software may be subject to
foreign copyright.  Permission in the United States and in foreign countries,
to the extent that NIST may hold copyright, to use, copy, modify, create
derivative works, and distribute this software and its documentation without
fee is hereby granted on a non-exclusive basis, provided that this notice and
disclaimer of warranty appears in all copies.

THE SOFTWARE IS PROVIDED 'AS IS' WITHOUT ANY WARRANTY OF ANY KIND, EITHER
EXPRESSED, IMPLIED, OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, ANY WARRANTY
THAT THE SOFTWARE WILL CONFORM TO SPECIFICATIONS, ANY IMPLIED WARRANTIES OF
MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE, AND FREEDOM FROM
INFRINGEMENT, AND ANY WARRANTY THAT THE DOCUMENTATION WILL CONFORM TO THE
SOFTWARE, OR ANY WARRANTY THAT THE SOFTWARE WILL BE ERROR FREE.  IN NO EVENT
SHALL NIST BE LIABLE FOR ANY DAMAGES, INCLUDING, BUT NOT LIMITED TO, DIRECT,
INDIRECT, SPECIAL OR CONSEQUENTIAL DAMAGES, ARISING OUT OF, RESULTING FROM, OR
IN ANY WAY CONNECTED WITH THIS SOFTWARE, WHETHER OR NOT BASED UPON WARRANTY,
CONTRACT, TORT, OR OTHERWISE, WHETHER OR NOT INJURY WAS SUSTAINED BY PERSONS
OR PROPERTY OR OTHERWISE, AND WHETHER OR NOT LOSS WAS SUSTAINED FROM, OR AROSE
OUT OF THE RESULTS OF, OR USE OF, THE SOFTWARE OR SERVICES PROVIDED HEREUNDER.
"""
import rospy
import tf2_ros
import moveit_commander as mc

from geometry_msgs.msg import TransformStamped
from nist_gear.msg import Order, Model, LogicalCameraImage, VacuumGripperState

from std_srvs.srv import Trigger
from nist_gear.srv import AGVControl, AGVToAssemblyStation, GetMaterialLocations, VacuumGripperControl

import sys
import copy
import yaml
import re
from math import pi, sqrt


def start_competition():
    """ Start the competition through ROS service call """

    rospy.wait_for_service('/ariac/start_competition')
    rospy.ServiceProxy('/ariac/start_competition', Trigger)()


def end_competition():
    """ End the competition through ROS service call"""

    rospy.wait_for_service('/ariac/end_competition')
    rospy.ServiceProxy('/ariac/end_competition', Trigger)()


def submit_kitting_shipment(agv, assembly_station, shipment_type):
    """ ROS service call to submit a kitting shipment

    Args:
    assembly_station (str): The name of the assembly station where the shipment should be delivered
    shipment_type (str): Type of shipment, which is retrieved from the topic /ariac/orders

    Returns:
    bool: status of the service call
    """

    rospy.wait_for_service('/ariac/' + agv + '/submit_shipment')
    rospy.ServiceProxy('/ariac/' + agv + '/submit_shipment', AGVToAssemblyStation)(assembly_station, shipment_type)


def submit_assembly_shipment(assembly_station, shipment_type):
    """ ROS service call to submit an assembly shipment"""

    rospy.wait_for_service('/ariac/' + assembly_station + '/submit_shipment')
    rospy.ServiceProxy('/ariac/' + assembly_station + '/submit_shipment', AssemblyStationSubmitShipment)(shipment_type)


def get_order():
    """ Get the current order from the /ariac/orders topic"""

    order = rospy.wait_for_message('/ariac/orders', Order)
    return order


def get_part_type_location(part):
    """ Get vessels where a specific part type can be found. This function will not work in competition mode. """

    rospy.wait_for_service('/ariac/material_locations')
    response = rospy.ServiceProxy('/ariac/material_locations',
                                  GetMaterialLocations)(part.type)
    reachable_location = None
    for loc in response.storage_units:
        if 'bin' in loc.unit_id:
            reachable_location = loc.unit_id
            break
    assert(reachable_location), "This implementation only reaches bins"
    return reachable_location


def get_parts_from_cameras():
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)

    # wait for all cameras to be broadcasting
    all_topics = rospy.get_published_topics()
    camera_topics = [t for t, _ in all_topics if '/ariac/logical_camera' in t]
    for topic in camera_topics:
        rospy.wait_for_message(topic, LogicalCameraImage)

    camera_frame_format = r"logical_camera_[0-9]+_(\w+)_[0-9]+_frame"
    all_frames = yaml.safe_load(tf_buffer.all_frames_as_yaml()).keys()
    part_frames = [f for f in all_frames if re.match(camera_frame_format, f)]

    objects = []
    for frame in part_frames:
        try:
            world_tf = tf_buffer.lookup_transform(
                'world',
                frame,
                rospy.Time(),
                rospy.Duration(0.1)
            )
            ee_tf = tf_buffer.lookup_transform(
                frame,
                'ee_link',
                rospy.Time(),
                rospy.Duration(0.1)
            )
        except (tf2_ros.LookupException, tf2_ros.ExtrapolationException) as e:
            continue

        # remove stale transforms
        tf_time = rospy.Time(
            world_tf.header.stamp.secs,
            world_tf.header.stamp.nsecs
        )
        if rospy.Time.now() - tf_time > rospy.Duration(1.0):
            continue

        model = Model()
        model.type = re.match(camera_frame_format, frame).group(1)
        model.pose.position = world_tf.transform.translation
        model.pose.orientation = ee_tf.transform.rotation
        objects.append(model)
    return objects


def get_target_world_pose(target, agv):
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)
    tf_broadcaster = tf2_ros.StaticTransformBroadcaster()
    h_frame = ''

    if agv == 'agv1':
        h_frame = 'kit_tray_1'
    elif agv == 'agv2':
        h_frame = 'kit_tray_2'
    elif agv == 'agv3':
        h_frame = 'kit_tray_3'
    elif agv == 'agv4':
        h_frame = 'kit_tray_4'

    tf_msg = TransformStamped()
    if not h_frame:
        tf_msg.header.frame_id = h_frame
    else:
        assert(h_frame), "No AGV provided"

    tf_msg.header.stamp = rospy.Time()
    tf_msg.child_frame_id = 'target_frame'
    tf_msg.transform.translation = target.pose.position
    tf_msg.transform.rotation = target.pose.orientation

    for _ in range(5):
        tf_broadcaster.sendTransform(tf_msg)

    # tf lookup fails occasionally, this automatically retries the lookup
    MAX_ATTEMPTS = 10
    attempts = 0
    while attempts < MAX_ATTEMPTS:
        try:
            world_target_tf = tf_buffer.lookup_transform(
                'world',
                'target_frame',
                rospy.Time(),
                rospy.Duration(0.1)
            )
            ee_target_tf = tf_buffer.lookup_transform(
                'target_frame',
                'ee_link',
                rospy.Time(),
                rospy.Duration(0.1)
            )
            break
        except:
            continue

    world_target = copy.deepcopy(target)
    world_target.pose.position = world_target_tf.transform.translation
    world_target.pose.orientation = ee_target_tf.transform.rotation
    return world_target


class MoveitRunner():

    def __init__(self, group_names, node_name='ariac_moveit_example', ns='', robot_description='robot_description'):
        mc.roscpp_initialize(sys.argv)
        rospy.init_node(node_name, anonymous=True)

        self.robot = mc.RobotCommander(ns+'/'+robot_description, ns)
        self.scene = mc.PlanningSceneInterface(ns)
        self.groups = {}
        for group_name in group_names:
            group = mc.MoveGroupCommander(
                group_name,
                robot_description=ns+'/'+robot_description,
                ns=ns
            )
            group.set_goal_tolerance(0.05)
            self.groups[group_name] = group

        self.set_preset_location()
        self.goto_preset_location('start')

    def set_preset_location(self):
        '''
        Define preset locations for easy navigation.

        For the kitting robot, the order of the joints are:
        - linear_arm_actuator_joint
        - shoulder_pan_joint
        - shoulder_lift_joint
        - elbow_joint
        - wrist_1_joint
        - wrist_2_joint
        - wrist_3_joint

        For the gantry_torso group, the joints are:
        - small_long_joint
        - torso_base_main_joint
        - torso_main_torso_tray_joint,
        - torso_rail_joint

        For the gantry_arm group, the joints are:
        - gantry_arm_elbow_joint
        - gantry_arm_shoulder_lift_joint
        - gantry_arm_shoulder_pan_joint
        - gantry_arm_vacuum_gripper_joint
        - gantry_arm_wrist_1_joint
        - gantry_arm_wrist_2_joint
        - gantry_arm_wrist_3_joint
        '''

        locations = {}

        name = 'home'
        kitting_arm = [0, 3.141594222190707, -1.128743290405139, 1.5106304587276407, 4.25, -1.5079761953269788, 0]
        gantry_torso = [0, 0, 0]
        gantry_arm = [0.0, -1.13, 1.88, -0.72, 1.55, 0.83]
        locations[name] = (kitting_arm, gantry_torso, gantry_arm)

        name = 'bin8'
        kitting_arm = [1.3458887656258813, -0.5601138939850792, -0.2804510290896989, 0, -0.8072468824120538, 1.5385783777411373, 0.8298981409931709]
        gantry_torso = [-2.48400006879773, -1.6336322021423504, 0, 3.4200004668605506]
        gantry_arm = [0.0, -pi/4, pi/2, -pi/4, pi/2, 0]
        locations[name] = (kitting)

        name = 'standby'
        kitting_arm = [2.70, 3.141594222190707, -1.01, 1.88, 3.77, -1.55, 0]
        gantry_torso = [0, 0, 0]
        gantry_arm = [0.0, -pi/4, pi/2, -pi/4, pi/2, 0]
        locations[name] = (kitting)

        name = 'agv4'
        kitting_arm = [1.50, 3.141594222190707, -1.01, 1.88, 3.77, -1.55, 0]
        gantry_torso = [0, 0, 0]
        gantry_arm = [0.0, -pi/4, pi/2, -pi/4, pi/2, 0]
        locations[name] = (kitting)

        self.locations = locations

    def goto_preset_location(self, location_name, robot_type):

        group = None
        if robot_type == 'kitting_robot':
            group = self.groups['kitting_arm']
        elif robot_type == 'gantry_robot':
            group = self.groups['gantry_full']

        kitting_arm, gantry_torso, gantry_arm = self.locations[location_name]
        location_pose = group.get_current_joint_values()

        if kitting:
            location_pose[:] = kitting
            print("Location Pose:", location_pose)

        # If the robot controller reports a path tolerance violation,
        # this will automatically re-attempt the motion
        MAX_ATTEMPTS = 5
        attempts = 0
        while not group.go(location_pose, wait=True):
            attempts += 1
            assert(attempts < MAX_ATTEMPTS)

    def move_part(self, part, target, part_location, agv, robot_type):
        """
        Pick a part from a bin and place it in the tray of an AGV
        Args:
        part (str): The name of the assembly station where the shipment should be delivered
        target (str): Type of shipment, which is retrieved from the topic /ariac/orders

        Returns:
        bool: status of the service call
        """

        # This example only uses the kitting robot only
        group = self.groups['kitting_arm']
        gm = GripperManager(ns='/ariac/kitting/arm/gripper/')

        near_pick_pose = copy.deepcopy(part.pose)
        pick_pose = copy.deepcopy(part.pose)
        place_pose = copy.deepcopy(target.pose)

        near_pick_pose.position.z += 0.1
        pick_pose.position.z += 0.050
        place_pose.position.z += 0.1

        print("part_pose: ", part)
        print("near_pick_pose: ", near_pick_pose.position.z)
        self.goto_preset_location(part_location)
        gm.activate_gripper()

        path = [near_pick_pose, pick_pose]
        self.cartesian_move(group, path)

        num_attempts = 0
        MAX_ATTEMPTS = 20
        while not gm.is_object_attached() and num_attempts < MAX_ATTEMPTS:
            num_attempts += 1
            rospy.sleep(0.1)

        if not gm.is_object_attached():
            self.goto_preset_location(part_location)
            self.goto_preset_location('standby')
            self.goto_preset_location('start')
            return False

        self.goto_preset_location('standby')
        self.goto_preset_location(agv)

        path = [place_pose]
        self.cartesian_move(group, path)

        gm.deactivate_gripper()

        self.goto_preset_location('standby')
        self.goto_preset_location('start')
        return True

    def cartesian_move(self, group, waypoints):
        print("CARTESIAN MOVE")
        (plan, fraction) = group.compute_cartesian_path(waypoints, 0.01, 0.0)
        group.execute(plan, wait=True)


class GripperManager():
    def __init__(self, ns):
        self.ns = ns

    def activate_gripper(self):
        rospy.wait_for_service(self.ns + 'control')
        rospy.ServiceProxy(self.ns + 'control', VacuumGripperControl)(True)

    def deactivate_gripper(self):
        rospy.wait_for_service(self.ns + 'control')
        rospy.ServiceProxy(self.ns + 'control', VacuumGripperControl)(False)

    def is_object_attached(self):
        status = rospy.wait_for_message(self.ns + 'state', VacuumGripperState)
        return status.attached


if __name__ == '__main__':

    # all moveit groups defined for both robots
    kitting_group_names = ['kitting_arm']
    gantry_group_names = ['gantry_full', 'gantry_arm', 'gantry_torso']

    # an instance of MoveitRunner for the kitting robot
    moveit_runner_kitting = MoveitRunner(kitting_group_names, ns='/ariac/kitting')
    # an instance of MoveitRunner for the gantry robot
    moveit_runner_gantry = MoveitRunner(gantry_group_names, ns='/ariac/gantry')

    start_competition()
    order = get_order()
    # print(order)
    agv_states = {'agv1': [], 'agv2': [], 'agv3': [], 'agv4': []}

    all_known_parts = get_parts_from_cameras()
    # print(all_known_parts)

    for shipment in order.shipments:
        if shipment.agv_id == 'any':
            active_agv = 'agv1'
        else:
            active_agv = shipment.agv_id

        agv_state = agv_states[active_agv]

        while True:
            valid_products = []
            for product in shipment.products:
                if product not in agv_state:
                    valid_products.append(product)

            candidate_moves = []
            for part in all_known_parts:
                for product in valid_products:
                    if part.type == product.type:
                        candidate_moves.append((part, product))

            if candidate_moves:
                part, target = candidate_moves[0]

                world_target = get_target_world_pose(target, active_agv)
                part_location = get_part_type_location(part)

                print("world_target: ", world_target)
                print("part_location: ", part_location)

                move_successful = moveit_runner.move_part(
                    part,
                    world_target,
                    part_location,
                    active_agv
                )
                if move_successful:
                    all_known_parts.remove(part)
                    agv_state.append(target)
            else:
                break

        submit_shipment(active_agv, shipment.shipment_type)
        agv_states[active_agv] = []

    end_competition()
    print('Done')
