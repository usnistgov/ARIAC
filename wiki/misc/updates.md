Wiki | [Home](../../README.md) | [Documentation](../documentation/documentation.md) | [Tutorials](../tutorials/tutorials.md) | [Qualifiers](../qualifiers/qualifier.md) | [Finals](../finals/finals.md)

-------------------------------------------------

## Released 2021, Mar 13

- This new update should fix most of the tickets created on Github. Please, close tickets for which you have received answers or for which the original issues are fixed in this new update.
- Updated the [Installation](../tutorials/installation.md#install-ros-and-gazebo) page to prevent segmentation fault from happening when launching GEAR.
  - The link to the installation instructions have been moved to the Home page of the wiki.
- Added a cost for the camera mounted on the robot. Used properly, this camera can greatly reduce the number of logical cameras in the environment. Therefore, we added a cost of $600 for this camera.
  - Competitors now have the ability to enable/disable this camera in the user configuration file (see [Sensor Interface](../tutorials/sensor_interface.md#camera-mounted-on-the-robot)).
- We are working on increasing the field of view of the logical camera (while keeping its current cost). Once done, you will be able to cover, for instance, `agv1`, `agv2`, `bin1`, `bin2`, `bin3`, `bin4` (see new field of view [here](../tutorials/sensor_interface.md#logical-camera)).
- Fixed some typos in the documentation.
- Added more explanations in the documentation to answer questions asked on Github.
- Scoring formulas updated for both assembly and kitting (see [Scoring](../documentation/scoring.md)).
- Penalty for parts dropped on the floor has been implemented (see [Scoring](../documentation/scoring.md#trial-score-ts)).
- Added a topic (`/ariac/agv#/station` which reports the location of each AGV in the workcell (see [Competition Interface](../documentation/competition_interface_documentation.md#process-management)).

<!-- - Added a Python test competitor which does both kitting and assembly (see `ariac2021_example.py` in the `scripts` directory). -->

## Released 2021, Feb 28

- Many bug fixes.
- [Github ticket](https://github.com/usnistgov/ARIAC/issues/37) cleared on not activating the conveyor belt when parts are not spawned on the conveyor belt.
- Ticket on [part slipping](https://github.com/usnistgov/ARIAC/issues/52) on the gantry tray "resolved". For now, competitors can lock/unlock parts with the use of a service. We will see how to automatically lock/unlock parts.
- Added trial configuration files for each agility challenge (**NOTE**: The structure of the [config](../../nist_gear/config) directory has been slightly modified).
- Services to ship AGVs and briefcases during development were added.
- Announce a new order (priority = 1) when an AGV is submitted to an assembly station (see [GEAR Interface](../tutorials/gear_interface.md), [Agility Challenges](../documentation/agility_challenges.md), and [YAML Configuration Files](../documentation/configuration_files.md)).
- In-process task change to force a robot breakdown based on world state (see [Agility Challenges](../documentation/agility_challenges.md)).
- Competitors have to discard faulty products using the faulty product collector, which has a deletion wall (see [Competition Specifications](../documentation/competition_specifications.md)).
  - The floor is lava: Parts falling on the floor will automatically be removed by the simulation. This plugin will be used to detect products falling on the floor and points will be removed if those parts are faulty. We are currently discussing about the point removal aspect.
- Resized some parts so they can be placed in briefcases with less tolerance.
- Each section of the wiki is now documented (except some sections about qualifiers and finals).

## Released 2021, Feb 21

- Updates including more examples of trial configuration files and complete documentation.
- Some Github tickets (created for ARIAC 2020) will be cleared this week.
- Some new found bugs with the assembly robot tray are also being fixed.
  
<!-- 
- Solutions for fixing playback on host machines (issue [#28](https://github.com/usnistgov/ARIAC/issues/28)) are provided [here](https://github.com/usnistgov/ARIAC/blob/master/wiki/tutorials/automated_evaluation.md#playing-back-the-simulation) 

## Released 2020, April 21

- Updated scoring plugin to take into account the score for part color.
- Updated docker script files https://github.com/usnistgov/ariac-docker
- Updated docker images which now take into account part color. Get the new docker images with pull_dockerhub_images.bash


## Released 2020, April 4

- Changed the wiki documentation on scoring (see [here](../documentation/scoring.md)).
  - Baseline cost (**BC**) increased from 1700 to 10000.
  - Collision (**COL**) is 0 if arms collide with each other, with the torso, or if the robot collides with a moving obstacle.
  - New condition for scoring added for using the correct part color.
- We have extended the deadline of the qualifiers round (from Apr 10th to Apr 24th).
- We added a [note](../documentation/automated_evaluation.md) specifying that the option `development-mode` should not be used during the qualifiers and finals, i.e., this [option](https://github.com/usnistgov/ARIAC/blob/bf77a0c61520f5d3a80e004c825a5045c4eeaca6/nist_gear/launch/sample_environment.launch#L23) should be removed).


## Released 2020, March 21 -->

<!-- - Added qual A yaml files (see [here](../qualifiers/qualifier_scenarios.md)).
- Added challenge yaml files (see [here](../documentation/agility_challenges.md)).
  - Major changes to the wiki, which is now  up to date (except for [this page](../tutorials/gear_interface.md)) .
  - Please contact me (zeid.kootbally@nist.gov) if anything is out of place.

## Released 2020, March 20

- Changed `product_` to `part_` in yaml trial config files.

- For instance, the following code:

  ```yaml
  products:
        product_0:
          type: piston_rod_part_blue
          pose:
            xyz: [0.1, -0.1, 0]
            rpy: [0, 0, 0]
  ```

- is replaced with:

```yaml
products:
      part_0:
        type: piston_rod_part_blue
        pose:
          xyz: [0.1, -0.1, 0]
          rpy: [0, 0, 0]
```



## Released 2020, March 13

- Cost of the RGBD camera is set to be the same as the Logical camera ($500). -->
