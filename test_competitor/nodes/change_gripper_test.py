#!/usr/bin/env python

import rospy
import moveit_commander as mc
from std_msgs.msg import String
import yaml
import os
import sys
from test_competitor.gripper_manager import GripperManager
from test_competitor.competitor import Competitor


def get_gantry_gripper():
    """
    get_gantry_gripper _summary_

    Returns:
        str: type of the gripper
    """
    gripper = rospy.wait_for_message('/ariac/gantry/arm/gripper/type', String)
    return gripper.data


class GantryGripperTest():

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
                self.goto_preset_location('home', "gantry_robot")
            elif 'kitting' in key:
                # print("GROUPS", self.groups)
                self.goto_preset_location('home', "kitting_robot")

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

    def goto_preset_location(self, location_name, robot_type):

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


def main():

    # Define MoveIt groups
    gantry_group_names = ['gantry_full', 'gantry_arm', 'gantry_torso']

    moveit_runner_gantry = GantryGripperTest(
        gantry_group_names, ns='/ariac/gantry')
    moveit_runner_gantry.gantry_status_publisher.publish("init")

    competitor = Competitor()
    # Start the competition
    competitor.start_competition()

    moveit_runner_gantry.swap_gripper("gripper_tray")
    moveit_runner_gantry.go_home()


if __name__ == '__main__':
    main()
