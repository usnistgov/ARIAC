Wiki | [Home](../../README.md) | [Documentation](../documentation/documentation.md) | [Tutorials](../tutorials/tutorials.md) | [Qualifiers](../qualifiers/qualifier.md) | [Finals](../finals/finals.md)

---

## Release 2022, April 30

- Added new fix for issues with movable trays.
- Note: There is no need to use the service `/ariac/kit_tray_1/lock` before submitting a kit tray nor before the use of `/ariac/kit_tray_N/move_to_station`. Before submission or before moving to a station the movable tray and parts in the movable tray will be automatically locked.

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
