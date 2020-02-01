#!/usr/bin/env python

"""
Author: Pavel Piliptchak
Contact: pavel.piliptchak@nist.gov

This software was developed by employees of the National Institute of Standards and Technology (NIST), an agency of the Federal Government and is being made available as a public service. Pursuant to title 17 United States Code Section 105, works of NIST employees are not subject to copyright protection in the United States.  This software may be subject to foreign copyright.  Permission in the United States and in foreign countries, to the extent that NIST may hold copyright, to use, copy, modify, create derivative works, and distribute this software and its documentation without fee is hereby granted on a non-exclusive basis, provided that this notice and disclaimer of warranty appears in all copies.

THE SOFTWARE IS PROVIDED 'AS IS' WITHOUT ANY WARRANTY OF ANY KIND, EITHER EXPRESSED, IMPLIED, OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, ANY WARRANTY THAT THE SOFTWARE WILL CONFORM TO SPECIFICATIONS, ANY IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE, AND FREEDOM FROM INFRINGEMENT, AND ANY WARRANTY THAT THE DOCUMENTATION WILL CONFORM TO THE SOFTWARE, OR ANY WARRANTY THAT THE SOFTWARE WILL BE ERROR FREE.  IN NO EVENT SHALL NIST BE LIABLE FOR ANY DAMAGES, INCLUDING, BUT NOT LIMITED TO, DIRECT, INDIRECT, SPECIAL OR CONSEQUENTIAL DAMAGES, ARISING OUT OF, RESULTING FROM, OR IN ANY WAY CONNECTED WITH THIS SOFTWARE, WHETHER OR NOT BASED UPON WARRANTY, CONTRACT, TORT, OR OTHERWISE, WHETHER OR NOT INJURY WAS SUSTAINED BY PERSONS OR PROPERTY OR OTHERWISE, AND WHETHER OR NOT LOSS WAS SUSTAINED FROM, OR AROSE OUT OF THE RESULTS OF, OR USE OF, THE SOFTWARE OR SERVICES PROVIDED HEREUNDER.
"""
import rospy
import tf2_ros

import moveit_commander
import moveit_msgs.msg, geometry_msgs.msg, nist_gear.msg, nist_gear.srv, std_srvs.srv


import sys, copy, yaml, re
from math import pi, sqrt


class MoveitRunner():
    def __init__(self, group_names, node_name='ariac_moveit_example', ns='', robot_description='robot_description'):
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node(node_name, anonymous=True)

        self.robot = moveit_commander.RobotCommander(robot_description=ns+'/'+robot_description, ns=ns)
        self.scene = moveit_commander.PlanningSceneInterface(ns=ns)

        self.groups = {}
        for group_name in group_names:
            self.groups[group_name] = moveit_commander.MoveGroupCommander(group_name, robot_description=ns+'/'+robot_description, ns=ns)
            self.groups[group_name].set_goal_tolerance(0.05)

        self.define_preset_locations()
        self.goto_preset_location('start')

    def define_preset_locations(self):
        locations = {}
        
        name = 'start'
        gantry = [0, 0, 0]
        left = [0.0, -pi/4, pi/2, -pi/4, pi/2, 0]
        right = [pi, -pi/4, pi/2, -pi/4, pi/2, 0]
        locations[name] = (gantry, left, right)

        name = 'bin3'
        gantry = [4.0, -1.1, 0.]
        left = [0.0, 0.0, pi/4, 0, pi/2, 0]
        right = None
        locations[name] = (gantry, left, right)

        name = 'bin4'
        gantry = [5.0, -1.2, 0.]
        left = [0.0, 0.0, pi/4, 0, pi/2, 0]
        right = None
        locations[name] = (gantry, left, right)

        name ='shelf1'
        gantry = [2.5, -2.3, 0.]
        left = [-pi/2, -pi, -pi/2, -pi/2, 0, 0]
        right = None
        locations[name] = (gantry, left, right)

        name = 'standby'
        gantry = None
        left = [0.0, -pi/4, pi/2, -pi/4, pi/2, 0]
        right = [pi, -pi/4, pi/2, -pi/4, pi/2, 0]
        locations[name] = (gantry, left, right)

        name = 'agv1'
        gantry = [0., -6.5, pi/2]
        left = [0.0, 0.0, pi/4, 0, pi/2, 0]
        right = None
        locations[name] = (gantry, left, right)

        name = 'agv2'
        gantry = [0., 6.5, -pi/2]
        left = [0.0, 0.0, pi/4, 0, pi/2, 0]
        right = None
        locations[name] = (gantry, left, right)

        self.locations = locations

    def goto_preset_location(self, location_name):
        group = self.groups['Full_Robot']
        gantry, left, right = self.locations[location_name]
        location_pose = group.get_current_joint_values()
        
        if gantry:
            location_pose[:3] = gantry
        if left:
            location_pose[3:-6] = left
        if right:
            location_pose[-6:] = right

        # If the robot controller reports a path tolerance violation,
        # this will automatically re-attempt the motion
        MAX_ATTEMPTS = 10
        attempts = 0
        while not group.go(location_pose, wait=True):
            attempts += 1
            assert(attempts < MAX_ATTEMPTS)
    
    def move_part(self, part, target, part_location, agv):
        group = self.groups['Left_Arm']
    
        near_pick_pose = copy.deepcopy(part.pose)
        pick_pose = copy.deepcopy(part.pose)
        place_pose = copy.deepcopy(target.pose)

        near_pick_pose.position.z += 0.1
        pick_pose.position.z += 0.015
        place_pose.position.z += 0.1
    
        rospy.wait_for_service('/ariac/gantry/left_arm/gripper/control')
        gripper_enable = rospy.ServiceProxy('/ariac/gantry/left_arm/gripper/control', nist_gear.srv.VacuumGripperControl)
    
        self.goto_preset_location(part_location)
        gripper_enable(True)
    
        (plan, fraction) = group.compute_cartesian_path([near_pick_pose, pick_pose], 0.01, 0.0)
        group.execute(plan, wait=True)
    
        num_grasp_attempts = 0
        while not rospy.wait_for_message('/ariac/gantry/left_arm/gripper/state', nist_gear.msg.VacuumGripperState).attached and num_grasp_attempts < 20:
            num_grasp_attempts += 1
            rospy.sleep(0.1)

        if num_grasp_attempts >= 20:
            self.goto_preset_location(part_location)
            self.goto_preset_location('standby')
            self.goto_preset_location('start')
            return False
    
        if 'shelf' in part_location:
            self.goto_preset_location(part_location)
        self.goto_preset_location('standby')
        self.goto_preset_location('start')
        self.goto_preset_location(agv)
    
        (plan, fraction) = group.compute_cartesian_path([place_pose], 0.01, 0.0)
        group.execute(plan, wait=True)
        gripper_enable(False)
    
        self.goto_preset_location('standby')
        self.goto_preset_location('start')
        return True

def get_parts_from_cameras():
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)

    # wait for all cameras to be broadcasting
    camera_topics = [topic for topic,_ in rospy.get_published_topics() if '/ariac/logical_camera' in topic]
    for topic in camera_topics:
        rospy.wait_for_message(topic, nist_gear.msg.LogicalCameraImage)

    all_frames = yaml.safe_load(tf_buffer.all_frames_as_yaml()).keys()
    part_frames = [frame for frame in all_frames if re.match(r"logical_camera_[0-9]+_(\w+)_[0-9]+_frame",frame)]

    objects = []
    for frame in part_frames:
        try:
            world_tf = tf_buffer.lookup_transform('world', frame, rospy.Time(), rospy.Duration(0.1))
            # get the orientation that the gripper needs in order to grasp the part
            ee_tf = tf_buffer.lookup_transform(frame, 'left_ee_link', rospy.Time(), rospy.Duration(0.1))
        except (tf2_ros.LookupException, tf2_ros.ExtrapolationException) as e:
            continue 
        
        # remove stale transforms
        tf_time = rospy.Time(world_tf.header.stamp.secs, world_tf.header.stamp.nsecs)
        if rospy.Time.now() - tf_time > rospy.Duration(1.0):
            continue

        model = nist_gear.msg.Model()
        model.type = re.match(r"logical_camera_[0-9]+_(\w+)_[0-9]+_frame",frame).group(1)
        model.pose.position = world_tf.transform.translation
        model.pose.orientation = ee_tf.transform.rotation

        objects.append(model)
    return objects

def get_target_world_pose(target, agv):
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)
    tf_broadcaster = tf2_ros.StaticTransformBroadcaster()
    
    tf_msg = geometry_msgs.msg.TransformStamped()
    tf_msg.header.frame_id = 'kit_tray_1' if agv=='agv1' else 'kit_tray_2'
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
            world_target_tf = tf_buffer.lookup_transform('world', 'target_frame', rospy.Time(), rospy.Duration(1.0))
            ee_target_tf = tf_buffer.lookup_transform('target_frame', 'left_ee_link', rospy.Time(), rospy.Duration(0.1))
            break
        except:
            continue
    
    world_target = copy.deepcopy(target)
    world_target.pose.position = world_target_tf.transform.translation
    world_target.pose.orientation = ee_target_tf.transform.rotation
    return world_target

def get_order():
    order = rospy.wait_for_message('/ariac/orders', nist_gear.msg.Order)
    return order

def process_shipment(current_order, agv_state):
    valid_products = []
    for product in shipment.products:
        if product not in agv_state:
            valid_products.append(product)
    return valid_products

def get_part_type_location(part):
    rospy.wait_for_service('/ariac/material_locations')
    response = rospy.ServiceProxy('/ariac/material_locations', nist_gear.srv.GetMaterialLocations)(part.type)
    reachable_location = None
    for loc in response.storage_units:
        if 'shelf' in loc.unit_id or 'bin' in loc.unit_id:
            reachable_location = loc.unit_id
            break
    assert(reachable_location), "This implementation can only grasp shelf and bin parts"
    return reachable_location

def submit_shipment(agv, name):
    rospy.wait_for_service('/ariac/' + agv)
    rospy.ServiceProxy('/ariac/' + agv, nist_gear.srv.AGVControl)(name)


if __name__ == '__main__':

    group_names = ['Full_Robot', 'Left_Arm', 'Right_Arm', 'Gantry']
    moveit_runner = MoveitRunner(group_names, ns='/ariac/gantry')

    rospy.wait_for_service('/ariac/start_competition')
    rospy.ServiceProxy('/ariac/start_competition', std_srvs.srv.Trigger)()

    current_order = get_order()
    agv_states = {'agv1':[], 'agv2':[]}
    
    all_known_parts = get_parts_from_cameras()

    for shipment in current_order.shipments:
       active_agv = 'agv1' if shipment.agv_id == 'agv1' else 'agv2'
       active_agv_state = agv_states[active_agv]
    
       while True:
           valid_products = process_shipment(shipment, active_agv_state)
           candidate_moves = [(part, product) for part in all_known_parts for product in valid_products if part.type == product.type]
           if candidate_moves:
               part,target = candidate_moves[0]

               world_target = get_target_world_pose(target, active_agv)
               part_location = get_part_type_location(part)
               move_successful = moveit_runner.move_part(part, world_target, part_location, active_agv)
               if move_successful:
                   all_known_parts.remove(part)
                   active_agv_state.append(target)
           else:
               break

       submit_shipment(active_agv, shipment.shipment_type)
       agv_states[active_agv] = []
    
    rospy.wait_for_service('/ariac/end_competition')
    rospy.ServiceProxy('/ariac/end_competition', std_srvs.srv.Trigger)()
    print('Done')
