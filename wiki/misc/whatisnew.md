Wiki | [Home](../../README.md) | [Documentation](../documentation/documentation.md) | [Tutorials](../tutorials/tutorials.md) | [Qualifiers](../qualifiers/qualifier.md) | [Finals](../finals/finals.md)

---

- [Wiki | What is new in ARIAC 2022?](#wiki--what-is-new-in-ariac-2022)
  - [ARIAC Developers](#ariac-developers)
  - [Overview](#overview)
  - [Cameras on Robot](#cameras-on-robot)
  - [Automated Guided Vehicles (AGVs)](#automated-guided-vehicles-agvs)
  - [Robots](#robots)
  - [Humans](#humans)
  - [Movable Trays](#movable-trays)
  - [Drop Locations](#drop-locations)
  - [Gripper Change](#gripper-change)
  - [ROS Services, Topics, and Messages](#ros-services-topics-and-messages)
  - [Test Competitors](#test-competitors)

# Wiki | What is new in ARIAC 2022?


## ARIAC Developers


First, we apologize for the late release of ARIAC 2022. We spent a fair amount of time implementing new challenges and fixing previous bugs.


We also have a new person on the team, Justin Albrecht (justin.albrecht@nist.gov), whose roles were crucial to the release of ARIAC. Pavel Piliptchak (pavel.piliptchak@nist.gov) was also heavily involved in the release of the software and provided an implementation of the assembly plugin.


## Overview

- The rendered simulation environment is mostly the same from ARIAC 2022. Only a gripper changing station and two tray tables have been added.
- Many new challenges were added to ARIAC 2022.
- The assembly plugin is now working in ARIAC 2022. Competitors will now be tasked to assemble 4 parts to build ventilators.

## Cameras on Robot

- Competitors now have the ability to use two cameras mounted on the gantry torso.
  - One camera is placed above the gantry tray so competitors can see parts located on the tray.
  - Another camera has been placed below the tray, thus allowing competitors to visualize the content in bins.
- Vision processing is required with these cameras.
- Each camera has a cost and competitors have the choice to use one of the cameras, both, or none.

## Automated Guided Vehicles (AGVs)

- AGVs can now be moved between stations as often competitors wish or as needed. It is now possible to build the same shipment multiple times on the same AGV.
- Two different service types will move the AGVs:
  - A service to submit an AGV moves the AGV to an assembly station and the kit on the AGV is evaluated.
  - A service to move an AGV to a different station with no evaluation.

## Robots

- A robot mounted on a linear rail (aka **kitting robot**) can only perform kitting.
- A gantry robot mounted on the ceiling (aka **assembly robot**) can perform both kitting and assembly.
- We are working on some technical issues to allow the GEAR interface to deactivate  one of the two robots during kitting. This year the robots will be disabled in the sense that their controllers will stop working. This will force competitors to use the other robot to perform a new task or to finish an existing task (i.e., take over the task the broken robot was doing).

## Humans

HUMANS ARE BACK!!11!1!

Trials with humans consist of having humans located at either assembly station 2 (`as2`) or assembly station 4 (`as4`), or both.

The gantry robot must wait for humans to move away from these assembly stations before performing assembly.

## Movable Trays

Movable trays are part of ARIAC 2022. In the past, competitors built kits directly on the back of AGVs (on static kit trays, e.g., `kit_tray_1`). This year, competitors (the gantry robot) will need to get a movable tray from one of the two tray tables, place the movable tray on an AGV static kit tray, and place parts in the movable tray. There are different types of movable trays, each announced order includes the type of movable trays needed for shipments.

## Drop Locations

The faulty gripper challenge can now happen when picking up parts from bins. In previous competitions, this challenge  only applied when placing parts. This year, it can happen after pick AND during place.

## Gripper Change

- Changing gripper is one of the newest challenges in ARIAC 2022. This challenge only applies to the gantry robot.
- There are two types of gripper that the gantry robot can use:
  - `gripper_part` can only grasp parts (any part).
  - `gripper_tray` can only grasp movable trays (any movable tray).
  - The gripper plugin prevents `gripper_part` to be used on movable trays and `gripper_tray` to be used on parts.
- To change gripper, competitors need to move the gantry over the gripper changing station and change gripper through a ROS service. You will not see anything fancy, just new data being published on a topic.
- At the beginning of a trial, one of these two grippers will be mounted on the robot. This is specified in the `options` field in a trial configuration file.

## ROS Services, Topics, and Messages

- A few more ROS services, topics, and messages were added to interact with GEAR. They are explain in the [API Section](../documentation/api.md).

## Test Competitors

ARIAC developers have provided 2 important test competitors. A test competitor is basically an example on how to perform kitting and assembly. These test competitors can be found in the package **test_competitor**. Competitors are free to use and modify these files. Hopefully, inline comments and docstring documentations provided in the source files will be helpful. 

**Disclaimer**: These test competitors are provided as examples. They are for from being perfect but we believe they will greatly help newcomers.

* There is a demo showing the use of OpenCV to detect parts located in bins and on the gantry tray. This demo showcases the 2 cameras mounted on the gantry. The node for this demo is `object_detector_node.py` located in the package **test_competitor**.
* To see a demo of assembly with all parts, run the commands below in separate terminals. The argument `station` is required to specify where assembly must be performed. The different values are `as1`, `as2`, `as3`, and `as4`. You should see the robot picking up 4 parts from an AGV and placing them in the ventilator.

```bash
$ roslaunch test_competitor assembly.launch station:=as1
$ rosrun test_competitor assembly_commander_node.py
```

* To see a demo of kitting, run the commands below in separate terminals. You should see the gantry robot picking up a movable tray and placing it on an AGV, followed by the kitting robot completing the kit.

```bash
$ roslaunch test_competitor kitting.launch
$ rosrun test_competitor kitting_commander_node.py
```

Wiki | [Home](../../README.md) | [Documentation](../documentation/documentation.md) | [Tutorials](../tutorials/tutorials.md) | [Qualifiers](../qualifiers/qualifier.md) | [Finals](../finals/finals.md)
