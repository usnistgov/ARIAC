-------------------------------------------------
- Wiki | [Home](../../README.md) | [Documentation](../documentation/documentation.md) | [Tutorials](../tutorials/tutorials.md) | [Qualifiers](../qualifiers/qualifier.md) | [Finals](../finals/finals.md)
-------------------------------------------------

## Released 2020, May 09
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


## Released 2020, March 21

- Added qual A yaml files (see [here](../qualifiers/qualifier_scenarios.md)).
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

- Cost of the RGBD camera is set to be the same as the Logical camera ($500).
