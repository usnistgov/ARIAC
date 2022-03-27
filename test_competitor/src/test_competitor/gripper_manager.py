#!/usr/bin/env python

import rospy
from nist_gear.srv import VacuumGripperControl, ChangeGripper
from nist_gear.msg import VacuumGripperState


class GripperManager():
    def __init__(self, ns):
        self.ns = ns

    def change_gripper(self, gripper_type):
        """
        Attach the gripper gripper_type to the gantry

        Args:
            gripper_type (str): Type of the gripper (gripper_tray or gripper_part)

        Returns:
            bool: Service execution result
        """
        rospy.wait_for_service(self.ns + 'change')
        try:
            change = rospy.ServiceProxy(self.ns + 'change', ChangeGripper)
            result = change(gripper_type)
            rospy.loginfo(result.message)
            return result.success
        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)

    def activate_gripper(self):
        """
        Activate a robot's gripper to grasp objects

        Returns:
            bool: Service execution result
        """
        rospy.wait_for_service(self.ns + 'control')
        try:
            control = rospy.ServiceProxy(
                self.ns + 'control', VacuumGripperControl)
            result = control(True)
            return result.success
        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)

    def deactivate_gripper(self):
        """
        Deactivate a robot's gripper to release objects

        Returns:
            bool: Service execution result
        """
        rospy.wait_for_service(self.ns + 'control')
        try:
            control = rospy.ServiceProxy(
                self.ns + 'control', VacuumGripperControl)
            result = control(False)
            return result.success
        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)

    def is_object_attached(self):
        status = rospy.wait_for_message(self.ns + 'state', VacuumGripperState)
        return status.attached
