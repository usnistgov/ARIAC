#!/usr/bin/env python

import math
from copy import deepcopy
import rospy
import tf2_ros
import PyKDL
import moveit_commander
from geometry_msgs.msg import Pose, Quaternion, Vector3, TransformStamped
from tf.transformations import euler_from_quaternion
from tf_conversions import posemath
from std_msgs.msg import String
from std_srvs.srv import Trigger
from nist_gear.msg import LogicalCameraImage, VacuumGripperState
from nist_gear.srv import VacuumGripperControl, VacuumGripperControlRequest


class AssemblyPart:
    def __init__(self, part_type, color, pose):
        self.part_type = part_type
        self.color = color
        self.pose = pose


class AssemblyCommander:
    ee_transform = PyKDL.Frame(PyKDL.Rotation.RPY(
        0, math.pi/2, math.pi), PyKDL.Vector(0, 0, 0))
    gripper_offset = 0.013

    grip_offsets = {'sensor': PyKDL.Frame(PyKDL.Rotation.RPY(0, 0, 0), PyKDL.Vector(0.01, 0, 0.035 + gripper_offset)),
                    'pump': PyKDL.Frame(PyKDL.Rotation.RPY(0, 0, 0), PyKDL.Vector(0, 0, 0.058 + gripper_offset)),
                    'regulator': PyKDL.Frame(PyKDL.Rotation.RPY(0, 0, 0), PyKDL.Vector(0, 0, 0.034 + gripper_offset)),
                    'battery': PyKDL.Frame(PyKDL.Rotation.RPY(0, 0, 0), PyKDL.Vector(0, 0, 0.030 + gripper_offset))}

    def __init__(self):
        self.camera_pose = {}
        self.models = {}
        self.as1_kit_tray_sub = rospy.Subscriber('ariac/as1_kit_tray', LogicalCameraImage,
                                                 callback=self.as1_kit_tray_callback)

        self.as2_kit_tray_sub = rospy.Subscriber('ariac/as2_kit_tray', LogicalCameraImage,
                                                 callback=self.as2_kit_tray_callback)

        self.as3_kit_tray_sub = rospy.Subscriber('ariac/as3_kit_tray', LogicalCameraImage,
                                                 callback=self.as3_kit_tray_callback)

        self.as4_kit_tray_sub = rospy.Subscriber('ariac/as4_kit_tray', LogicalCameraImage,
                                                 callback=self.as4_kit_tray_callback)

        self.part_attached = False
        self.gripper_state_sub = rospy.Subscriber('/ariac/gantry/arm/gripper/state', VacuumGripperState,
                                                  callback=self.gripper_state_callback)

        # Create TF Listener
        self.tfBuffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tfBuffer)
        self.tf_broadcaster = tf2_ros.StaticTransformBroadcaster()

        ns = "ariac/gantry"
        robot_description = ns + "/robot_description"

        self.torso_move_group = moveit_commander.MoveGroupCommander('gantry_torso',
                                                                    robot_description=robot_description,
                                                                    ns=ns)
        self.torso_move_group.set_max_acceleration_scaling_factor(0.2)

        self.arm_move_group = moveit_commander.MoveGroupCommander('gantry_arm',
                                                                  robot_description=robot_description,
                                                                  ns=ns)

        # Create gripper service proxy
        self.actuate_gripper = rospy.ServiceProxy(
            '/ariac/gantry/arm/gripper/control', VacuumGripperControl)
        self.actuate_req = VacuumGripperControlRequest()

        # Set up callbacks to know where the agvs are
        self.agv1_location = None
        self.agv1_location_sub = rospy.Subscriber(
            'ariac/agv1/station', String, callback=self.agv1_callback)

        self.agv2_location = None
        self.agv2_location_sub = rospy.Subscriber(
            'ariac/agv2/station', String, callback=self.agv2_callback)

        self.agv3_location = None
        self.agv3_location_sub = rospy.Subscriber(
            'ariac/agv3/station', String, callback=self.agv3_callback)

        self.agv4_location = None
        self.agv4_location_sub = rospy.Subscriber(
            'ariac/agv4/station', String, callback=self.agv4_callback)

    def get_pose_on_kit_tray(self, station_id, part):
        """Use logical camera to determine information about the part. Return part pose in world coordinates. 
        Returns None if the specified part is not on the desired kit_tray"""

        rate = rospy.Rate(10)
        while not self.models[station_id]:
            rate.sleep()

        for model in self.models[station_id]:
            _, part_type, color = model.type.split("_")
            if part_type == part.part_type and color == part.color:
                # Convert to world coordinates
                f1 = posemath.fromMsg(self.camera_pose[station_id])
                f2 = posemath.fromMsg(model.pose)

                return posemath.toMsg(f1*f2)

        return False

    def move_arm_to_home_position(self):
        """Commands the arm to move specified home position where it won't hit anything while the gantry moves"""

        self.arm_move_group.set_joint_value_target(
            [0, -math.pi, math.pi-0.6, math.pi+0.6, -math.pi/2, 0])
        self.arm_move_group.go()
        rospy.sleep(0.5)

    def move_gantry_to_station(self, station_id):
        """Commands the gantry to move in front of the briefcase for the current assembly"""

        # Move gantry to center aisle
        try:
            t = self.tfBuffer.lookup_transform(
                'world', 'torso_base', rospy.Time(), rospy.Duration(1)).transform
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logerr("Unable to lookup transform")
            return False
        self.move_gantry(t.translation.x, 0, 0)
        rospy.sleep(0.5)

        # Move gantry to correct x position
        try:
            t = self.tfBuffer.lookup_transform(
                'world', "briefcase_" + station_id[-1], rospy.Time(), rospy.Duration(1)).transform
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logerr("Unable to lookup transform")
            return False

        self.move_gantry(t.translation.x + 2.5, 0, 0)
        rospy.sleep(0.5)

        # Move gantry to correct y position and rotation
        if station_id == 'as1' or station_id == 'as3':
            self.move_gantry(t.translation.x + 2, t.translation.y, 0)
        elif station_id == 'as2' or station_id == 'as4':
            self.move_gantry(t.translation.x + 2, t.translation.y, math.pi)

    def move_gantry_to_agv(self, station_id):
        """Commands the gantry to move beside the agv"""

        # Check that the agv is at the correct location
        correct_location = True
        if station_id == 'as1':
            if not self.agv1_location == 'as1':
                correct_location = False
        if station_id == 'as2':
            if not self.agv2_location == 'as2':
                correct_location = False
        if station_id == 'as3':
            if not self.agv3_location == 'as3':
                correct_location = False
        if station_id == 'as4':
            if not self.agv4_location == 'as4':
                correct_location = False

        if not correct_location:
            rospy.logerr("agv is not in the correct location")
            return False

        tray = "kit_tray_" + station_id[-1]

        try:
            agv_transform = self.tfBuffer.lookup_transform(
                'world', tray, rospy.Time(), rospy.Duration(1)).transform
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logerr("Unable to lookup transform")
            return False

        # Move gantry to correct position
        if station_id == 'as1' or station_id == 'as3':
            self.move_gantry(agv_transform.translation.x,
                             agv_transform.translation.y - 1.5, 0)
        elif station_id == 'as2' or station_id == 'as4':
            self.move_gantry(agv_transform.translation.x,
                             agv_transform.translation.y + 1.5, math.pi)

        rospy.sleep(0.5)

        return True

    def pick_part(self, station_id, tray_pose, part):
        """Commands the gantry arm to pickup the requested part"""

        rospy.loginfo('Picking up a ' + part.color + '_' + part.part_type)

        # Define the pick pose
        q = [tray_pose.orientation.x, tray_pose.orientation.y,
             tray_pose.orientation.z, tray_pose.orientation.w]
        rotation = euler_from_quaternion(q)[2]

        tray_frame = PyKDL.Frame(PyKDL.Rotation.RPY(0, 0, rotation),
                                 PyKDL.Vector(tray_pose.position.x, tray_pose.position.y, tray_pose.position.z))

        pick_pose = posemath.toMsg(
            tray_frame*self.grip_offsets[part.part_type]*self.ee_transform)

        # Move gantry so that end effector is above part
        ee_pose = self.arm_move_group.get_current_pose().pose
        gantry_joints = self.torso_move_group.get_current_joint_values()
        gantry_joints[0] += pick_pose.position.x - ee_pose.position.x
        self.torso_move_group.set_joint_value_target(gantry_joints)
        self.torso_move_group.go()
        rospy.sleep(0.5)

        ee_pose = self.arm_move_group.get_current_pose().pose
        gantry_joints = self.torso_move_group.get_current_joint_values()

        if station_id == 'as1' or station_id == 'as3':
            gantry_joints[1] -= pick_pose.position.y - ee_pose.position.y - 0.1
        elif station_id == 'as2' or station_id == 'as4':
            gantry_joints[1] -= pick_pose.position.y - ee_pose.position.y + 0.1

        self.torso_move_group.set_joint_value_target(gantry_joints)
        self.torso_move_group.go()
        rospy.sleep(0.5)

        # Define pick_approach poses
        ee_pose = self.arm_move_group.get_current_pose().pose

        delta_y = pick_pose.position.y - ee_pose.position.y
        delta_z = pick_pose.position.z - ee_pose.position.z

        approach_1 = deepcopy(ee_pose)
        approach_1.position.y += delta_y
        approach_1.position.z += delta_z + 0.8

        approach_2 = deepcopy(approach_1)
        approach_2.position.z -= 0.6

        approach_3 = deepcopy(pick_pose)
        approach_3.position.z += 0.2

        self.publish_transform(approach_1, "approach_1", "world")
        self.publish_transform(approach_2, "approach_2", "world")
        self.publish_transform(approach_3, "approach_3", "world")
        self.publish_transform(pick_pose, "pick", "world")

        # Move through approach poses
        self.arm_move_group.set_pose_target(approach_1)
        plan = self.arm_move_group.plan()
        self.arm_move_group.execute(plan, wait=True)

        self.arm_move_group.set_pose_target(approach_2)
        plan = self.arm_move_group.plan()
        self.arm_move_group.execute(plan, wait=True)

        self.arm_move_group.set_pose_target(approach_3)
        plan = self.arm_move_group.plan()
        self.arm_move_group.execute(plan, wait=True)

        # Move straight down to pick pose
        plan, _ = self.arm_move_group.compute_cartesian_path(
            [pick_pose], 0.01, 0.0)
        self.arm_move_group.execute(plan, wait=True)

        # Turn on gripper
        self.gripper_on()

        rospy.sleep(0.5)

        # Keep moving down by a mm until the part is successfully picked up
        while not self.part_attached:
            pick_pose.position.z -= .001
            plan, _ = self.arm_move_group.compute_cartesian_path(
                [pick_pose], 0.001, 0.0)
            self.arm_move_group.execute(plan, wait=True)
            rospy.sleep(0.5)

        # Move through waypoints in reverse
        plan, _ = self.arm_move_group.compute_cartesian_path(
            [approach_3], 0.01, 0.0)
        self.arm_move_group.execute(plan, wait=True)

    def move_gantry_to_assembly_position(self, briefcase, part):
        """Commands the gantry to move beside the requested briefcase, in the correct position to assembly the 
        requested part"""
        try:
            t_briefcase = self.tfBuffer.lookup_transform(
                'world', briefcase, rospy.Time(), rospy.Duration(2)).transform
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logerr("Unable to lookup transform")
            return False

        if part.part_type == 'sensor':
            gantry_x = t_briefcase.translation.x + 1.8
            gantry_y = t_briefcase.translation.y - 0.3
            gantry_rot = math.pi/2
        elif part.part_type == 'regulator':
            gantry_x = t_briefcase.translation.x + 1.6
            gantry_y = t_briefcase.translation.y + 0.2
            gantry_rot = math.pi/2
        elif part.part_type == 'battery':
            gantry_x = t_briefcase.translation.x + 1.2
            gantry_y = t_briefcase.translation.y - 0.8
            gantry_rot = math.pi/4
        elif part.part_type == 'pump':
            gantry_x = t_briefcase.translation.x + 1.2
            gantry_y = t_briefcase.translation.y - 1
            gantry_rot = math.pi/4

        self.move_gantry(gantry_x, gantry_y, gantry_rot)

    def assemble_part(self, briefcase, part):
        """Command the gantry arm to orient and assemble the part"""

        rospy.loginfo('Assembling the ' + part.color + '_' + part.part_type)

        try:
            t = self.tfBuffer.lookup_transform(
                'world', briefcase, rospy.Time(), rospy.Duration(1)).transform
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logerr("Unable to lookup transform")
            return False

        # Convert transform to kdl frame
        briefcase_pose = Pose()
        briefcase_pose.position = Vector3(
            t.translation.x, t.translation.y, t.translation.z)
        briefcase_pose.orientation = Quaternion(
            t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w)

        f1 = posemath.fromMsg(briefcase_pose)
        f2 = posemath.fromMsg(part.pose)

        assembly_pose = posemath.toMsg(f1*f2)

        self.publish_transform(assembly_pose, 'assembly_pose', 'world')

        if part.part_type == "sensor":
            assembly_approach = deepcopy(assembly_pose)
            assembly_approach.position.x += 0.2
        else:
            assembly_approach = deepcopy(assembly_pose)
            assembly_approach.position.z += 0.2

        # Create pose for the robot end effector
        ee_assembly_pose = posemath.toMsg(posemath.fromMsg(
            assembly_pose)*self.grip_offsets[part.part_type]*self.ee_transform)
        ee_assembly_approach = posemath.toMsg(posemath.fromMsg(
            assembly_approach)*self.grip_offsets[part.part_type]*self.ee_transform)

        self.publish_transform(ee_assembly_pose, 'ee_assembly_pose', 'world')
        self.publish_transform(ee_assembly_approach,
                               'ee_assembly_approach', 'world')

        if part.part_type == "sensor":
            # Move gantry so that end effector is in-line with part
            ee_pose = self.arm_move_group.get_current_pose().pose
            gantry_joints = self.torso_move_group.get_current_joint_values()
            gantry_joints[1] -= ee_assembly_approach.position.y - \
                ee_pose.position.y
            self.torso_move_group.set_joint_value_target(gantry_joints)
            self.torso_move_group.go()
            rospy.sleep(1)

            # Move to initial assembly approach
            ee_assembly_approach2 = deepcopy(ee_assembly_approach)
            ee_assembly_approach2.position.z += 0.35
            self.arm_move_group.set_pose_target(ee_assembly_approach2)
            self.arm_move_group.go()
            rospy.sleep(1)

        # Move to assembly approach
        self.arm_move_group.set_pose_target(ee_assembly_approach)
        plan = self.arm_move_group.plan()

        if plan.joint_trajectory.points:
            self.arm_move_group.execute(plan)
        else:
            rospy.logerr(
                "Unable to generate trajectory to reach assembly approach.")
            return False

        rospy.sleep(0.5)

        # Move to assembly pose
        plan, _ = self.arm_move_group.compute_cartesian_path(
            [ee_assembly_pose], 0.01, 0.0)

        plan = self.arm_move_group.retime_trajectory(
            self.arm_move_group.get_current_state(), plan, 0.05, 0.1)

        self.arm_move_group.execute(plan, wait=True)

        # Turn off the gripper
        self.gripper_off()
        rospy.sleep(1)

        # Move robot arm away
        after_assembly = deepcopy(ee_assembly_pose)

        if part.part_type == "sensor":
            after_assembly.position.x += 0.2
            after_assembly.position.y -= 0.2
        elif part.part_type == "regulator":
            after_assembly.position.x += 0.2
        else:
            after_assembly.position.z += 0.2

        self.arm_move_group.set_pose_target(after_assembly)
        self.arm_move_group.go()

        return True

    def move_gantry(self, x, y, rotation):
        try:
            t = self.tfBuffer.lookup_transform(
                'world', 'torso_base', rospy.Time(), rospy.Duration(1)).transform
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logerr("Unable to lookup transform")
            return False

        joints = self.torso_move_group.get_current_joint_values()

        joints[0] += x - t.translation.x
        joints[1] -= y - t.translation.y
        joints[2] = rotation

        self.torso_move_group.set_joint_value_target(joints)
        self.torso_move_group.go()

    def gripper_on(self):
        self.actuate_req.enable = True
        self.actuate_gripper(self.actuate_req)

    def gripper_off(self):
        self.actuate_req.enable = False
        self.actuate_gripper(self.actuate_req)

    def as1_kit_tray_callback(self, msg):
        self.camera_pose['as1'] = msg.pose
        self.models['as1'] = msg.models

    def as2_kit_tray_callback(self, msg):
        self.camera_pose['as2'] = msg.pose
        self.models['as2'] = msg.models

    def as3_kit_tray_callback(self, msg):
        self.camera_pose['as3'] = msg.pose
        self.models['as3'] = msg.models

    def as4_kit_tray_callback(self, msg):
        self.camera_pose['as4'] = msg.pose
        self.models['as4'] = msg.models

    def agv1_callback(self, msg):
        self.agv1_location = msg.data

    def agv2_callback(self, msg):
        self.agv2_location = msg.data

    def agv3_callback(self, msg):
        self.agv3_location = msg.data

    def agv4_callback(self, msg):
        self.agv4_location = msg.data

    def gripper_state_callback(self, msg):
        self.part_attached = msg.attached

    def publish_transform(self, pose, child, parent):
        t = TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = parent
        t.child_frame_id = child
        t.transform.translation.x = pose.position.x
        t.transform.translation.y = pose.position.y
        t.transform.translation.z = pose.position.z
        t.transform.rotation.x = pose.orientation.x
        t.transform.rotation.y = pose.orientation.y
        t.transform.rotation.z = pose.orientation.z
        t.transform.rotation.w = pose.orientation.w

        self.tf_broadcaster.sendTransform(t)
