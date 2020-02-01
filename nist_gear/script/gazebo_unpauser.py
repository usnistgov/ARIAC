#!/usr/bin/env python

# Software License Agreement (Apache License)
#
# Copyright 2016 Open Source Robotics Foundation
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

"""
Unpause gazebo after all models have been spawned.
"""

if __name__ == '__main__':
    import rospy
    from std_srvs.srv import Empty
    import time

    # Copied from https://github.com/ros-simulation/gazebo_ros_pkgs/blob/kinetic-devel/gazebo_ros/scripts/spawn_model
    rospy.logwarn("Waiting for unpause physics topic")
    rospy.wait_for_service('/gazebo/unpause_physics')
    # TODO(sloretz) check if all arm models spawned before unpausing
    rospy.logwarn("HACK sleeping before unpausing model")
    time.sleep(3)
    try:
      unpause_physics = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
      unpause_physics()
    except rospy.ServiceException as e:
        rospy.logerr("Unpause physics service call failed: %s", e)