Wiki | [Home](../../README.md) | [Documentation](../documentation/documentation.md) | [Tutorials](../tutorials/tutorials.md) | [Qualifiers](../qualifiers/qualifier.md) | [Finals](../finals/finals.md)

---

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
