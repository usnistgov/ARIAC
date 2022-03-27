#!/usr/bin/env python

import rospy 
import moveit_commander
from test_competitor.srv import MoveGantry, MoveGantryResponse

class GantryMover:
    def __init__(self):
        manipulator_name = "gantry_full"
        ns = "ariac/gantry"
        robot_description = ns + "/robot_description"
        
        self.move_group = moveit_commander.MoveGroupCommander(manipulator_name, robot_description=robot_description, ns=ns)
        
        # Advertise service
        self.move_gantry_srv = rospy.Service('move_gantry', MoveGantry, self.move_gantry)
        
    def move_gantry(self, req):
        current_joint_positions = self.move_group.get_current_joint_values()
    
        new_joint_positions = current_joint_positions
        new_joint_positions[0] += req.move_x
        new_joint_positions[1] += req.move_y
        
        self.move_group.set_joint_value_target(new_joint_positions)
        
        self.move_group.go()
        
        response = MoveGantryResponse()
        response.success = True
        
        return response
        
        
    