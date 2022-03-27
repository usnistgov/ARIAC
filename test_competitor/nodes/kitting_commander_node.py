#!/usr/bin/env python

import os

# from order import KittingShipment, MovableTray, Order
# nist

from nist_gear.srv import GetMaterialLocations
from nist_gear.msg import Orders, Model, LogicalCameraImage
from nist_gear.srv import AssemblyStationSubmitShipment, ChangeGripper
# ros
import rospy
import tf2_ros
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Pose
from geometry_msgs.msg import TransformStamped
from std_srvs.srv import Trigger
from std_msgs.msg import String
from tf.transformations import quaternion_multiply, quaternion_from_euler
# moveit
import moveit_commander as mc
from test_competitor.gripper_manager import GripperManager
from test_competitor.competitor import Competitor
# python
import sys
import copy
import yaml
import re
from math import pi, sqrt
import yaml
import io
from pprint import pprint


# pump: H, W, L: 0.125994,
# regulator: H: 0.068497 m, W: 0.101259, L: 0.133091
# Battery: H: 0.059587, L: 0.141396
# Sensor: H: 0.067675


def get_gantry_gripper():
    "Retrieve the type of gripper currently attached to the gantry robot"
    gripper = rospy.wait_for_message('/ariac/gantry/arm/gripper/type', String)
    return gripper.data


def get_order():
    """
    Receive one message from topic /ariac/orders

    This will create a new subscription to the topic,
    receive one message, then unsubscribe.
    It is better to have a regular subscriber to /ariac/orders
    to retrieve new orders.

    Returns:
        Orders: order published on /ariac/orders
    """

    order = rospy.wait_for_message('/ariac/orders', Orders)
    return order


def get_object_location(obj):
    """
    Get vessels where a specific object can be found.
    This function will not work in competition mode.

    Args:
        obj (Part or MovableTray): Find the bin or table
        where this object can be found.

    Returns:
        str: Bin or table where the object was found
    """

    # This service only works in --development-mode
    rospy.wait_for_service('/ariac/material_locations')
    response = rospy.ServiceProxy('/ariac/material_locations',
                                  GetMaterialLocations)(obj.type)
    reachable_location = None
    for loc in response.storage_units:
        # print(loc)
        if 'bin' or 'table' in loc.unit_id:
            reachable_location = loc.unit_id
            break
    # assert(reachable_location), "This implementation only reaches bins and tray tables"
    return reachable_location


def get_object_pose_in_workcell():
    """
   Get the world pose of each object found by cameras,
   including parts and movable trays

   Note, logical cameras must be named using the convention:
   logical_camera_x

    Returns:
        list: A list of all the objects found
    """
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)

    # wait for all cameras to be broadcasting
    all_topics = rospy.get_published_topics()
    #  NOTE: This will not work if your logical cameras are named differently
    camera_topics = [t for t, _ in all_topics if '/ariac/logical_camera' in t]
    for topic in camera_topics:
        rospy.wait_for_message(topic, LogicalCameraImage)

    # e.g., logical_camera_1_assembly_pump_red_1
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
        model.pose.orientation = world_tf.transform.rotation
        objects.append(model)
    return objects


def get_init_world_pose(part):
    """
    Get the pose of a part in a bin in the world frame

    Args:
        part (Part): part for which to get the pose
    """

    # print("PART TYPE", part.type)
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)

    # wait for all cameras to be broadcasting
    all_topics = rospy.get_published_topics()
    part_frames = []

    # for topic in all_topics:
    #     if part.type in topic:
    #         print("PART FOUND IN FRAME")
    #         rospy.wait_for_message(topic, LogicalCameraImage)

    all_frames = yaml.safe_load(tf_buffer.all_frames_as_yaml()).keys()
    for frame in all_frames:
        if part.type in frame:
            part_frames.append(frame)

    # #  NOTE: This will not work if your logical cameras are named differently
    camera_topics = [t for t, _ in all_topics if '/ariac/logical_camera' in t]
    for topic in camera_topics:
        rospy.wait_for_message(topic, LogicalCameraImage)

    camera_frame_format = r"logical_camera_[0-9]+_(\w+)_[0-9]+_frame"
    camera_frame_format = r"logical_camera_(\w+)_frame"
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
        # model.type = part_type
        model.pose.position = world_tf.transform.translation
        model.pose.orientation = world_tf.transform.rotation
        objects.append(model)
    return objects


def get_target_world_pose(target, agv):
    tf_buffer = tf2_ros.Buffer(rospy.Duration(3.0))
    tf2_ros.TransformListener(tf_buffer)
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
    tf_msg.header.frame_id = ''
    if h_frame:
        tf_msg.header.frame_id = h_frame
    else:
        assert(h_frame), "No AGV provided"

    tf_msg.header.stamp = rospy.Time()
    tf_msg.child_frame_id = 'target_frame'
    tf_msg.transform.translation = target.pose.position
    # print(tf_msg.transform.translation)
    tf_msg.transform.rotation = target.pose.orientation
    # print(tf_msg.transform.rotation)

    # Broadcast the frame target_frame as a child of h_frame
    for _ in range(5):
        tf_broadcaster.sendTransform(tf_msg)

    world_target_tf = TransformStamped()
    # Get the transform between world and target_frame

    for _ in range(20):
        try:
            world_target_tf = tf_buffer.lookup_transform(
                'world', 'target_frame', rospy.Time(), rospy.Duration(0.5))
        except (tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            rospy.logerr("Unable to lookup transform")

    world_target = copy.deepcopy(target)
    world_target.pose.position = world_target_tf.transform.translation
    world_target.pose.orientation = world_target_tf.transform.rotation
    return world_target


class MoveitRunner():

    def __init__(self, group_names, node_name='ariac_moveit_example', ns='',
                 robot_description='robot_description'):
        mc.roscpp_initialize(sys.argv)
        rospy.init_node(node_name, anonymous=True)

        self.gantry_status_publisher = rospy.Publisher(
            '/my/gantry_status', String, queue_size=10)
        self.gantry_status_subscriber = rospy.Subscriber(
            "/my/gantry_status", String, self.gantry_status_callback)
        self.gantry_status = ""

        self.robot = mc.RobotCommander(ns + '/' + robot_description, ns)
        self.scene = mc.PlanningSceneInterface(ns)
        self.groups = {}
        for group_name in group_names:
            group = mc.MoveGroupCommander(
                group_name,
                robot_description=ns + '/' + robot_description,
                ns=ns
            )
            group.set_goal_tolerance(0.05)
            self.groups[group_name] = group

        self.set_ariac_specs()
        self.go_home()

    def gantry_status_callback(self, msg):
        self.gantry_status = msg.data

    def go_home(self):
        for key in self.groups:

            if 'gantry' in key:
                # print("GROUPS", self.groups)
                self.goto_preset_location('home', robot_type="gantry_robot")
            elif 'kitting' in key:
                # print("GROUPS", self.groups)
                self.goto_preset_location('home', robot_type="kitting_robot")

    def load_preset_locations(self):
        """
        Return the path of the file containing specs for
        pick-and-place
        """
        # i.e. /path/to/dir/kitting_commander_node.py
        current_file = os.path.abspath(__file__)
        current_dir = os.path.split(current_file)[0]
        # print(current_dir)
        test_competitor_dir = os.path.split(current_dir)[0]
        specs_path = "config/robot_workcell_specs.yaml"
        return os.path.join(test_competitor_dir, specs_path)

    def set_ariac_specs(self):
        '''
        Static file with preset locations for easy navigation.
        This file also contains specs for bins that may be useful
        for pick-and-place
        '''

        ariac_specs_file = self.load_preset_locations()

        # Read YAML file
        with open(ariac_specs_file, 'r') as stream:
            data_loaded = yaml.safe_load(stream)

        locations = {}
        part_heights = {}
        bin_height = None
        agv_height = None

        for key, value in data_loaded.items():
            if key in "preset_locations":
                for loc, group in value.items():
                    kitting_arm = group['kitting_arm']
                    gantry_full = group['gantry_full']
                    gantry_torso = group['gantry_torso']
                    gantry_arm = group['gantry_arm']
                    locations[loc] = (
                        kitting_arm, gantry_full, gantry_torso, gantry_arm)
            if key in "bins":
                bin_height = value["height"]
            if key in "agvs":
                agv_height = value["height"]
            if key in "parts":
                for part, part_h in value.items():
                    part_name = part
                    part_height = part_h["height"]
                    part_heights[part_name] = part_height
                    # print(part_name, part_height)

        self.locations = locations
        self.part_heights = part_heights
        self.agv_height = agv_height
        self.bin_height = bin_height

    def goto_preset_location(self, location_name, robot_type="kitting_robot"):

        group = None
        if robot_type == 'kitting_robot':
            group = self.groups['kitting_arm']
        elif robot_type == 'gantry_robot':
            group = self.groups['gantry_full']

        # print(self.locations[location_name])

        kitting_arm, gantry_full, gantry_torso, gantry_arm = self.locations[location_name]
        location_pose = group.get_current_joint_values()
        # print("location_pose", location_pose)

        if robot_type == 'kitting_robot':
            location_pose[:] = kitting_arm
        elif robot_type == 'gantry_robot':
            location_pose[:] = gantry_full
            location_pose[:3] = gantry_torso
            location_pose[3:] = gantry_arm

        # If the robot controller reports a path tolerance violation,
        # this will automatically re-attempt the motion
        MAX_ATTEMPTS = 5
        attempts = 0
        while not group.go(location_pose, wait=True):
            attempts += 1
            assert(attempts < MAX_ATTEMPTS)

    def move_tray(self,
                  movable_tray_gripper,
                  tray_init_vessel,
                  tray_init_pose,
                  tray_target_pose):
        """
        Use the gantry robot to pick up a part from a table and place it on
        an agv.


        Args:
            movable_tray_gripper: gripper needed to move the tray
            tray_init_vessel: table on which the tray was located
            tray_init_pose: pose of the movable tray in world frame
            tray_target_pose: pose where to place the tray in world frame
        """

        # print(movable_tray_gripper, tray_init_vessel)
        gantry_torso_group = self.groups['gantry_torso']
        gantry_arm_group = self.groups['gantry_arm']
        gantry_full_group = self.groups['gantry_full']

        # group = self.groups['gantry_full']
        gantry_arm_group.set_goal_orientation_tolerance = 0.02
        gantry_arm_group.set_goal_position_tolerance = 0.02
        gm = GripperManager(ns='/ariac/gantry/arm/gripper/')

        # Check if a gripper change is needed
        current_gripper = get_gantry_gripper()
        # print(current_gripper, movable_tray_gripper)

        if str(current_gripper) != str(movable_tray_gripper):
            rospy.loginfo("Gripper change requested")
            if not self.swap_gripper(movable_tray_gripper):
                return

        self.goto_preset_location(tray_init_vessel, 'gantry_robot')

        ee_pose = gantry_arm_group.get_current_pose().pose

        # pre-grasp pose
        pre_grasp_pose = copy.deepcopy(tray_init_pose)
        pre_grasp_pose.orientation = ee_pose.orientation
        pre_grasp_pose.position.z += 0.05
        # print("pre_grasp_pose", pre_grasp_pose)

        # grasp pose
        grasp_pose = copy.deepcopy(tray_init_pose)
        grasp_pose.orientation = ee_pose.orientation
        grasp_pose.position.z += 0.02
        # print("grasp_pose", grasp_pose)

        gripper_status = False
        ACTIVATE_GRIPPER_ATTEMPT = 10
        gripper_attempt = 0
        while gripper_attempt < ACTIVATE_GRIPPER_ATTEMPT:
            gripper_attempt += 1
            gripper_status = gm.activate_gripper()
            # print("Gripper status", gripper_status)
            if gripper_status:
                break
            rospy.sleep(0.2)

        if not gm.activate_gripper():
            assert(gm.activate_gripper()), "Could not activate gripper"

        path = [pre_grasp_pose, grasp_pose]
        self.cartesian_move(gantry_arm_group, path)

        num_attempts = 0
        MAX_ATTEMPTS = 20
        while not gm.is_object_attached() and num_attempts < MAX_ATTEMPTS:
            num_attempts += 1
            rospy.sleep(0.1)

        if not gm.is_object_attached():
            self.goto_preset_location('home', 'gantry_robot')
            return False

        rospy.sleep(2.0)

        # current_pose
        current_ee_pose = gantry_arm_group.get_current_pose().pose
        current_pose = copy.deepcopy(current_ee_pose)
        current_pose.position.z += 0.02

        # post_grasp_pose
        post_grasp_pose = copy.deepcopy(current_ee_pose)
        post_grasp_pose.position.z += 0.05

        path = [current_pose, post_grasp_pose]
        self.cartesian_move(gantry_arm_group, path)

        # Move to the AGV to drop the tray
        # ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
        self.goto_preset_location('pre_agv1', 'gantry_robot')
        self.goto_preset_location('at_agv1', 'gantry_robot')

        # compute the relative rotation between tray pose and and pose on AGV
        rel_rot_q = self.compute_relative_rotation(
            tray_init_pose, tray_target_pose)

        # Compute the relative orientation of the gripper
        # between the initial pose of the tray and the
        # target pose of the tray
        q_current = quaternion_from_euler(0, 0, 0)
        q_current[0] = ee_pose.orientation.x
        q_current[1] = ee_pose.orientation.y
        q_current[2] = ee_pose.orientation.z
        q_current[3] = ee_pose.orientation.w

        ee_pose_q = quaternion_multiply(rel_rot_q, q_current)

        # pose of the tray on the AGV
        place_pose = copy.deepcopy(tray_target_pose)
        place_pose.orientation.x = ee_pose_q[0]
        place_pose.orientation.y = ee_pose_q[1]
        place_pose.orientation.z = ee_pose_q[2]
        place_pose.orientation.w = ee_pose_q[3]
        place_pose.position.z += 0.1

        path = [place_pose]
        self.cartesian_move(gantry_arm_group, path)

        gm.deactivate_gripper()

        self.goto_preset_location('pre_agv1', 'gantry_robot')
        self.goto_preset_location('home', 'gantry_robot')
        return True

    def swap_gripper(self, gripper_type):
        """
        Function to swap gripper on the gantry.

        Args:
            gripper_type: Gripper to attach to the gantry arm
        """
        group = self.groups['gantry_torso']
        gm = GripperManager(ns='/ariac/gantry/arm/gripper/')
        self.goto_preset_location('gripper_station', 'gantry_robot')
        rospy.Duration(2.0)

        return gm.change_gripper(gripper_type)

    def move_part(self,
                  part_init_vessel,
                  part_init_pose,
                  part_target_pose):

        # This example uses the kitting robot only
        group = self.groups['kitting_arm']
        gm = GripperManager(ns='/ariac/kitting/arm/gripper/')

        # compute the relative rotation between part pose in bin and tray
        rel_rot_q = self.compute_relative_rotation(
            part_init_pose.pose, part_target_pose.pose)

        # group.set_goal_orientation_tolerance = 0.02
        # group.set_goal_position_tolerance = 0.02

        # Define pick_approach poses
        ee_pose = group.get_current_pose().pose
        # pre-grasp pose
        near_pick_pose = copy.deepcopy(part_init_pose.pose)
        near_pick_pose.orientation = ee_pose.orientation
        near_pick_pose.position.z = 0.86 + 0.03
        # grasp pose
        pick_pose = copy.deepcopy(part_init_pose.pose)
        pick_pose.orientation = ee_pose.orientation
        pick_pose.position.z = 0.86

        gripper_status = gm.activate_gripper()
        if not gripper_status:
            assert(gm.activate_gripper()), "Could not activate gripper"

        MAX_ATTEMPT = 5
        current_attempt = 0
        while current_attempt < MAX_ATTEMPT and not gm.is_object_attached():
            current_attempt += 1
            self.goto_preset_location(part_init_vessel)
            path = [near_pick_pose, pick_pose]
            self.cartesian_move(group, path)

            MAX_GRASP_ATTEMPT = 5
            grasp_attempt = 0
            while not gm.is_object_attached() and grasp_attempt < MAX_GRASP_ATTEMPT:
                grasp_attempt += 1
                pick_pose.position.z -= .001
                plan, _ = group.compute_cartesian_path([pick_pose], 0.001, 0.0)
                group.execute(plan, wait=True)
                rospy.sleep(1.0)

        # Move through waypoints in reverse
        plan, _ = group.compute_cartesian_path(
            [pick_pose], 0.01, 0.0)
        group.execute(plan, wait=True)

        self.goto_preset_location(part_init_vessel)
        self.goto_preset_location('at_agv1')

        # ee_pose = group.get_current_pose().pose

        q_current = quaternion_from_euler(0, 0, 0)
        q_current[0] = ee_pose.orientation.x
        q_current[1] = ee_pose.orientation.y
        q_current[2] = ee_pose.orientation.z
        q_current[3] = ee_pose.orientation.w

        ee_pose_q = quaternion_multiply(rel_rot_q, q_current)

        # pose to place the part
        place_pose = copy.deepcopy(part_target_pose.pose)
        place_pose.orientation.x = ee_pose_q[0]
        place_pose.orientation.y = ee_pose_q[1]
        place_pose.orientation.z = ee_pose_q[2]
        place_pose.orientation.w = ee_pose_q[3]
        place_pose.position.z += 0.2

        path = [place_pose]
        self.cartesian_move(group, path)

        gm.deactivate_gripper()

        # self.goto_preset_location('standby')
        self.goto_preset_location('bin1')
        return True

    def cartesian_move(self, group, waypoints):
        (plan, fraction) = group.compute_cartesian_path(waypoints, 0.01, 0.0)
        group.execute(plan, wait=True)

    def compute_relative_rotation(self, init_pose, target_pose):
        """
        Compute the relative rotation between two poses.
        This relative rotation will be applied to the current end effector
        orientation.

        Args:
            init_pose (geometry_msgs.Pose): Pose of the part in the bin
            target_pose (geometry_msgs.Pose): Pose of the part in the tray
        """

        quat_init_inv = [init_pose.orientation.x,
                         init_pose.orientation.y,
                         init_pose.orientation.z,
                         -init_pose.orientation.w]  # Negate for inverse

        quat_target = [target_pose.orientation.x,
                       target_pose.orientation.y,
                       target_pose.orientation.z,
                       target_pose.orientation.w]

        q_relative_rotation = quaternion_multiply(quat_target, quat_init_inv)
        # print(q_relative_rotation, type(q_relative_rotation))

        return q_relative_rotation


def main():

    # Define MoveIt groups
    kitting_group_names = ['kitting_arm']
    gantry_group_names = ['gantry_full', 'gantry_arm', 'gantry_torso']

    # Instances of MoveitRunner for both robots
    # ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    # Do not modify the value of the ns parameters
    moveit_runner_kitting = MoveitRunner(
        kitting_group_names, ns='/ariac/kitting')
    moveit_runner_gantry = MoveitRunner(gantry_group_names, ns='/ariac/gantry')
    moveit_runner_gantry.gantry_status_publisher.publish("init")

    competitor = Competitor()
    # Start the competition
    competitor.start_competition()

    # Wait for order to be recieved
    r = rospy.Rate(10)
    while not competitor.received_order:
        r.sleep()

    # Initialize a dictionary of AGVs
    agv_states = {'agv1': [], 'agv2': [], 'agv3': [], 'agv4': []}
    # Get all objects in detected by logical cameras
    all_known_objects = get_object_pose_in_workcell()
    # assuming agv is any, we select agv1 to be the default agv
    active_agv = 'agv1'

    # Parse each order
    # ^^^^^^^^^^^^^^^^
    for order in competitor.orders:
        rospy.loginfo("Number of kitting shipment to complete: {0}".format(
            len(order.kitting_shipments)))

        for shipment in order.kitting_shipments:

            # retrieve the agv to use
            active_agv = shipment.agv
            agv_state = agv_states[active_agv]
            # retrieve the shipping type, e.g., order_0_kitting_0
            shipment_type = shipment.shipment_type
            # retrieve the assembly station to submit the agv
            assembly_station = shipment.assembly_station

            # retrieve the movable tray
            movable_tray_type = shipment.movable_tray.movable_tray_type
            # retrieve the pose of the movable tray on the AGV
            movable_tray_pose = shipment.movable_tray.pose
            # retrieve the gripper needed to handle the movable_tray
            movable_tray_gripper = shipment.movable_tray.gripper

            valid_trays = []
            # look for a movable tray in the workcell
            for tray in all_known_objects:
                if tray.type == movable_tray_type:
                    valid_trays.append(tray)

            if valid_trays:
                # vessel where the tray is located (table_1 or table_2)
                tray_init_vessel = get_object_location(valid_trays[0])
                # current pose of the tray in the world frame
                tray_init_pose = valid_trays[0].pose
                # pose of the movable tray in the agv (world frame)
                tray_target_pose = get_target_world_pose(
                    shipment.movable_tray, active_agv)

                # TASK GANTRY ROBOT TO GET MOVABLE TRAY
                # ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
                moveit_runner_gantry.move_tray(movable_tray_gripper,
                                               tray_init_vessel,
                                               tray_init_pose,
                                               tray_target_pose.pose)

            # TASK KITTING ROBOT TO GET THE PARTS
            # ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
            products = competitor.process_kitting_shipment(shipment)
            # list of unique product types
            unique_product_types = []

            for product in products:
                if product.type not in unique_product_types:
                    unique_product_types.append(product.type)

            # look for parts in the workcell with matching part type
            candidate_types = {}

            counter = 0

            for product_type in unique_product_types:
                counter = 0
                candidate_types[product_type] = {}
                candidate_parts = {}
                for part_in_workcell in all_known_objects:
                    if product_type == part_in_workcell.type:
                        # print(part_in_workcell)
                        counter += 1
                        candidate_parts[product_type +
                                        str(counter)] = [part_in_workcell]
                candidate_types[product_type] = candidate_parts

            # pprint(candidate_types)

            for product in products:
                _, product_init = candidate_types[product.type].popitem()
                product_init = product_init[0]
                # print("product_init", product_init)
                product_goal = get_target_world_pose(
                    product, active_agv)
                # print("product_goal", product_goal)
                product_vessel = get_object_location(product)
                # print("product_vessel", product_vessel)
                moveit_runner_kitting.move_part(
                    product_vessel,
                    product_init,
                    product_goal)

        competitor.submit_kitting_shipment(active_agv,
                                           assembly_station,
                                           shipment_type)
        # The following is not needed, ariac will end automatically
        # competitor.stop_competition()


if __name__ == '__main__':
    main()
