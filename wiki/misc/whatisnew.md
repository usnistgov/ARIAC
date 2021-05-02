Wiki | [Home](../../README.md) | [Documentation](../documentation/documentation.md) | [Tutorials](../tutorials/tutorials.md) | [Qualifiers](../qualifiers/qualifier.md) | [Finals](../finals/finals.md)

-------------------------------------------------

- [Wiki | What is new in ARIAC 2021?](#wiki--what-is-new-in-ariac-2021)
  - [Overview](#overview)
  - [Camera on Robot](#camera-on-robot)
  - [Automated Guided Vehicles (AGVs)](#automated-guided-vehicles-agvs)
  - [Robots](#robots)
  - [Parts](#parts)
  - [Tasks](#tasks)
  - [ROS Services, Topics, and Messages](#ros-services-topics-and-messages)

# Wiki | What is new in ARIAC 2021?

## Overview

- Shelves have been removed from ARIAC 2021. ARIAC developers spent a lot of time on shelf configurations, so expect them to be back in the next ARIAC iteration.
- Environment layout is more compact and less large than ARIAC 2020. It should take the gantry robot less time to navigate the workcell.
- In previous years, building kits on the wrong AGV was one of the situations that would nullified the trial score. This year, delivering the AGV to the wrong assembly station will also nullified the trial score.
- The orchestration for announcing new orders is more complex. See full description in the [Agility Challenges](../documentation/agility_challenges.md#new-order) page.

## Camera on Robot

- Competitors have the ability to add a depth camera on the robot torso to visualize the environment. Sensor processing is necessary to fully use this camera. See how to activate the camera on the robot in the [YAML Configuration Files](../documentation/configuration_files.md) (look for `enable_robot_camera`).
  
## Automated Guided Vehicles (AGVs)

- Two more AGVs were added, which makes a total of four AGVs for ARIAC 2021 (`agv1`, `agv2`, `agv3`, and `agv4`).
- Delivering AGVs has more importance than in previous years.
  - Delivering a kit is not done by sending the AGV away anymore. Competitors have to deliver AGVs to the correct assembly stations, even if no assembly is required afterwards.

## Robots

- A robot mounted on a linear rail (aka **kitting robot**) can only perform kitting.
- A gantry robot mounted on the ceiling (aka **assembly robot**) can perform both kitting and assembly.
- More information on robots and robot control can be found in the [Competition Specifications](../documentation/competition_specifications.md#3.-Robot) page and the [GEAR Interface](../tutorials/gear_interface.md) page.

## Parts

- Four new parts were created for both kitting and assembly. 
- Each part comes in 3 different colors (red, green, and blue).
- More information about parts can be found in the [Competition Specifications](../documentation/competition_specifications.md#2.4.-Parts) page.

## Tasks

- Kitting is one of the two tasks that can be performed in ARIAC 2021. This year, competitors have access to four automated guided vehicles (AGVs) to build kits. After building kits, AGVs must be sent to the correct assembly stations to be scored against the goal state.
- Assembly is introduced for the first time in ARIAC. Competitors need to pick up parts from AGVs or directly from bins/conveyor belt to do assembly. Once assembly is done, competitors can evaluate the final product.
  - Assembly is introduced for the first time. Assembly operations can be performed at four different assembly stations (`as1`, `as2`, `as3`, and `as4`).

More information about submitting Kitting and Assembly shipments can be found in the [GEAR Interface](../tutorials/gear_interface.md) page.

## ROS Services, Topics, and Messages

- A few more ROS services, topics, and messages were added to interact with GEAR. For instance, competitors can now check the health of the robots (whether they are enabled or disabled) through the topic `/ariac/robot_health`.

-------------------------------------------------
Wiki | [Home](../../README.md) | [Documentation](../documentation/documentation.md) | [Tutorials](../tutorials/tutorials.md) | [Qualifiers](../qualifiers/qualifier.md) | [Finals](../finals/finals.md)
