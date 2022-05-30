Wiki | [Home](../../README.md) | [Documentation](../documentation/documentation.md) | [Tutorials](../tutorials/tutorials.md) | [Qualifiers](../qualifiers/qualifier.md) | [Finals](../finals/finals.md)

---

## Release 2022, May 30
- **Note**: Make sure you do a clean rebuild of the software after a pull (`catkin clean -y && catkin build`)
- The segmentation fault that is reported in multiple tickets on github has been fixed. It is quite a bit long to explain the root cause of the issue but the way we fixed it was to not unlock the movable tray after it reaches the assembly station. When you submit an AGV to an assembly station, one of the Gazebo plugins locks the movable tray on the kit tray (slightly lifts the movable tray and attach it to the kit tray) and unlocks the movable tray when the AGV reaches the assembly station. The unlock mechanism triggered the segmentation fault. Since the movable tray is not unlocked anymore, you may need to adjust the z offset of the gantry arm for assembly.
  - Also, if you are using the ROS service to lock/unlock a movable tray on the kit tray, you can still lock the movable tray but do not use the service to unlock the movable tray.
- We have created a `finals` folder in the `trial_config`. It contains only one file for you to practice for the finals.
- Due to the time it take me to fix the segmentation fault issue, I am extending the submission deadline to 06/05.
- Today I will generate and push the docker image corresponding to the current version so you can practice the autoevaluation. I will replace the previous docker image on dockerhub (used for the qualifiers) with a new image.
- It may take a few hours for the image to be generated and uploaded.
## Release 2022, May 25

- **Note**: Make sure you do a clean rebuild of the software after a pull (`catkin clean -y && catkin build`)
- **Note**:  The status of an active and inactive robot published on `/ariac/robot_health` is 'active' and 'inactive', respectively. This used to be `True` or `False`.
- More information on what to expect in the finals:
  - Kitting:
    - The finals will consist of some scenarios where only kitting is required.
    - You need to make sure you can do kitting with both robots as the robot breakdown challenge will be part of some of the scenarios.
    - There will be high-priority orders where order_0 is kitting and order_1 is kitting.
    - Some scenarios require you do two shipments (two different AGVs).
    - All the other challenges will be present at least once.
      - Just a suggestion: Maybe it is better to flip a part on an empty agv than in an empty bin. The quality control sensor will tell you if the part is faulty before you start flipping it.
      - It is expected your robots keep working during sensor blackout. The goal is to minimize downtime. Also, human judges will score your performance based on videos we send them. You will get a lower score if they see your robots were idle during sensor blackout.
    - You need to make sure both robots can grasp parts from the conveyor belt (in case one of the two robots breaks down). See [complex_sample.yaml](../../nist_gear/config/trial_config/misc/complex_sample.yaml) for an example on how to spawn parts on the conveyor belt. Parts on the conveyor belt are all of the same type and color and are spawned in the middle of the belt. You should expect between 10 and 15 parts showing up on the belt.
  - Assembly:
    - The finals consist of some scenarios where only assembly is required.
    - There will be high-priority orders where order_0 is assembly and order_1 is kitting or assembly.
    - You can also have 2 shipments for assembly, which need to be performed at two different assembly stations.
    - The yaw of the pump and the battery inside the briefcase is either 0 or pi (do not assume the yaw is always 0).
    - The pose of the regular and the sensor will always be the same.
  - Kitting and Assembly:
    - Some scenarios in the finals consist of both kitting and assembly.
    - We can have situations where assembly is announced after you submit the AGV and assembly will need to be performed at the assembly station where the AGV was sent.
    - We can have scenarios with high-priority orders:
      - order_0 is assembly and order_1 is kitting.
      - order_0 is kitting and order_1 is assembly.
    - Remember we can have humans located at `as2` and `as4`. Can the gantry perform assembly at these stations with humans around?
## Release 2022, May 24

### Qualifiers Results
- The qualifiers have been evaluated and the output files have been uploaded in the folder of each team on Google Drive. In the qualifiers, we wanted to make sure your package could at least do pick and place. The qualifiers were evaluated with a new docker image where the following issues were fixed:
  - Issues with segmentation fault after assembly is done have been fixed. The reason of the segmentation fault was due to MoveitCommander object not being destroyed properly. This issue existed in ROS Indigo and for some reasons it is still in ROS Melodic. You can prevent the segmentation fault from happening by taking care of destroying the MoveitCommander object yourself. An example of how this is done can be found in [assembly_commander_node.py](../../nist_gear/test_competitor/nodes/../../../test_competitor/nodes/assembly_commander_node.py) (line 18) where a call to `rospy.on_shutdown(commander.shutdown)` is performed. **Note**: If you still have the segmentation fault, please make sure you called the moveit object destructor.
  - The scroring for assembly has been fixed. All the services and topics reporting the content of a briefcase should also work. **Note**: The test competitor code provided for assembly (`assembly_commander_node.py`) sometimes does not properly insert the regulator (incorrect pose but correct color and correct type). This code should be improved by competitors to achieve proper insertion. If competitors still have a hard time inserting the regulator, we will not include the regulator in the final scenarios for assembly.

- The files used in the qualifers can be found in the folder [qualifiers](../../nist_gear/config/trial_config/qualifiers/).
  - [qual_a.yaml](../../nist_gear/config/trial_config/qualifiers/qual_a.yaml) consists of kitting only.
  - [qual_b.yaml](../../nist_gear/config/trial_config/qualifiers/qual_b.yaml) consists of assembly only.

### Getting Ready for the Finals
- From now on, competitors should focus on getting ready for the finals using the current version of the github repo.
- Competitors must submit the auto evaluation file by **06/03 @5pm EST**.
- The docker image which will be used to evaluate the competitors' system during the final will be uploaded on https://hub.docker.com tomorrow and will be based on the current repository.
- Getting ready means to test your system with all the agility challenges.
- For high-priority orders, can your system handle the following scenarios?
  - order_0: assembly, order_1: kitting
  - order_0: assembly, order_1: assembly
  - order_0: kitting, order_1: kitting
  - order_0: kitting, order_1: assembly
- Can your system handle two regular orders where the second order is announced after the first order is shipped?
- The robot breakdown challenge has been fixed. Just a reminder that this challenge is about disabling the controllers of a robot during kitting. Instead of disabling the controllers right away, we are giving competitors 10 s to move the robot out of the way before the controllers are stopped. `robot_breakdown_sample.yaml` provides a scenario where one robot is disabled during kitting. `robot_breakdown_complex_sample.yaml` provides a scenario where one robot is disabled during kitting and re-enabled after the agv is shipped.
  - A reminder that the status of a robot is published on `/ariac/robot_health`. **Note**: We have modified the structure of [nist_gear/RobotHealth.msg](../../nist_gear/msg/RobotHealth.msg). The data type of robot status fields are now `string` instead of `bool`.


### Closing Tickets

There are many tickets that are open but should be closed. During the testing of the new version of the software, please take some time to close the tickets which have fixed issues.
  
## Release 2022, May 2

- We have considered some of your requests and we are extending the submission deadline to 5/7 at 5 PM EST.
## Release 2022, May 1

- A new folder was `ariac-docker` was added to the ARIAC package. It contains everything you need to get your package ready for the qualifiers.
- See the [automated evaluation](../documentation/automated_evaluation.md) for more information.

- Fixed ticket [#158](https://github.com/usnistgov/ARIAC/issues/158). Gripper change now works in both development and competition mode.
  - `roslaunch test_competitor gripper_test.launch` starts simulation and competition mode.
  - `rosrun test_competitor change_gripper_test.py` sends the gantry to the gripper station and a gripper change is successfully performed.
## Release 2022, April 30

- Added new fix for issues with movable trays.
- Note: There is no need to use the service `/ariac/kit_tray_1/lock` before submitting a kit tray nor before the use of `/ariac/kit_tray_N/move_to_station`. Before submission or before moving to a station the movable tray and parts in the movable tray will be automatically locked.
- Scripts and docker image will be released during the day.

## Release 2022, April 28

- Fixed issues with movable trays and parts falling of agv2, agv3, and agv4.
- Added a new service to check parts connected to a briefcase. See the [API](../documentation/api.md#process-management)
- Changed the service  `/ariac/kit_tray_X/get_content` to `/ariac/agvX/content`. Although the service `/ariac/kit_tray_X/get_content` is still advertised, do not use it. Now the movable tray and parts inside the movable tray are reported. See the [API](../documentation/api.md#cheats)
- Changed the submission date for the evaluation scripts. The new date is now Monday, 05/02 @5pm to give competitors time to test the new updates. If the updates show more stable results we will release the Docker image and the instructions for the evaluation scripts for the qualifiers on Saturday, 30th.
- **Information on the qualifiers**:
  - The qualifiers will consist of trials where kitting and assembly will be required.
  - The purpose of the qualifiers is to check that competitors made at least some efforts to do kitting and assembly.
  - During the qualifiers, the challenges will only include: flipped part, faulty parts, and sensor blackout.
  - We will address minor issues while you test the new updates.

## Release 2022, April 11
- Updated [wiki](../documentation/competition_specifications.md#movable-trays) to address questions posted in ticket [#133](https://github.com/usnistgov/ARIAC/issues/133).

## Release 2022, April 11
- Fixed AGVs x offset when shipped to assembly stations. 
  - Ticket [#139](https://github.com/usnistgov/ARIAC/issues/139)

## Release 2022, April 9

- Added a ROS service to lock/unlock a movable tray on an AGV. For instance, to lock/unlock a movable tray on AGV1, one can do:
  
    ```bash
    rosservice call /ariac/kit_tray_1/lock
    rosservice call /ariac/kit_tray_1/unlock
    ```

    The movable tray will slightly be lifted from the AGV, so make sure you modify your z position when placing a part on the movable tray. More information on this service can be found on the API page (see section [Process Management](../documentation/api.md#process-management)).

- Edited the Installation page (see section [Install ROS and Gazebo](../tutorials/installation.md#install-ros-and-gazebo)) to specify that **at least** Gazebo  9.16 is required.

- Updated [Readme.md](../../README.md#important-dates) with dates for the qualifiers and the finals.

## Release 2022, April 5

- Updated [scoring](../documentation/scoring.md) documentation with more information on movable trays.
## Release 2022, March 26


* Wiki released along with the software.
* Please report any broken links on the wiki at zeid.kootbally@gmail.com
