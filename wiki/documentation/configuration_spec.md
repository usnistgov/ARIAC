-------------------------------------------------
- Wiki | [Home](../../README.md) | [Documentation](../documentation/documentation.md) | [Tutorials](../tutorials/tutorials.md) | [Qualifiers](../qualifiers/qualifier.md) | [Finals](../finals/finals.md)
-------------------------------------------------

# Wiki | Documentation | Environment and Trials
- This page describes how to configure the simulated workcell environment by selecting and placing sensors.
- Additionally, how to configure the behavior/challenges of trials is described (only available for development, not available during the competition).
- The entire trial configuration is typically comprised of two configuration files:
   - the 'Trial configuration file' that details specifics of a particular trial of the competition, which users will not have control over, and
   - the 'Competitor configuration file' that details the placement of objects that users have control over.

- Both of these files are passed to an invocation of `gear.py` which parses the data and launches the appropriately configured simulation. The general format of the invocation is:


```
rosrun nist_gear gear.py -f <trial_config_file> <competitor_config_file>
```



# Competitor configuration file

* As a competitor, you are allowed to select the quantity, type, and location of sensors.
* Sensors can only be placed in static locations, they cannot be attached to the arm or otherwise be moved around the environment.

* Your choices must be written using the [YAML](http://yaml.org/) syntax in a configuration file.
* Here is an [example configuration file](https://github.com/usnistgov/ARIAC/blob/master/nist_gear/config/sample_user_config.yaml).

## How to add sensors

* The configuration YAML file contains a list of sensors denoted by the ``sensors:`` tag.
* Each sensor should have a **unique** name followed by the type of sensor and the sensor's position and orientation.

* Available sensor types include:
  * break_beam
  * proximity
  * logical_camera
  * laser_profiler
  * depth_camera
  * rgbd_camera
* A sensor's position and orientation is specified in global coordinates using and XYZ vector and Euler angles (roll, pitch, yaw).

* The following is the specification of one `break_beam` sensor


```
#!yaml
sensors:
  break_beam_1:
    type: break_beam
    pose:
      xyz: [1.6, 2.25, 0.95]
      rpy: [0, 0, 'pi']
```

* The following is the specification of both a `break_beam` and `logical_camera` sensor.

```
#!yaml

sensors:
  break_beam_1:
    type: break_beam
    pose:
      xyz: [1.6, 2.25, 0.95]
      rpy: [0, 0, 'pi']
  logical_camera_1:
    type: logical_camera
    pose:
      xyz: [1.21816, 3, 2]
      rpy: ['-pi', 'pi/2', 0]
```

## Visualizing sensor views
By default, the view of the sensors in the Gazebo simulation will not be displayed.
Enabling the sensor visualization may be useful while you are decided where to place sensors in the world.
You can enable sensor visualization by adding `--visualize-sensor-views` to the `gear.py` invocation.

## Reading sensor data
This is covered by the [sensor interface tutorial](../tutorials/sensor_interface.md).

# Competition configuration file
- Each trial of the competition is specified using a separate configuration file.
- A number of example trials are provided with the ARIAC software in the `config` directory.
- Each config file has a description of its behavior at the top of the file, e.g.:

```
#Number of orders: 1
#Number of shipments: 1
#Insufficiently many products: No
#Flipped products: No
#In-process order update: No
#Dropped products: Yes
#In-process order interruption: No
#Part flipping: No
#Faulty products: Yes
#Moving obstacles: 1
#Sensor blackout: Yes
#2 arms advantageous: Yes
#Time limit: No
```

These config files can be used to practice with the [various agility challenges](agility_challenges.md).

## Custom competition configuration files
- During the competition, competitors will not have control over the settings in this configuration file. 
- However, you may find that modifying the settings assists you during system development.
- Various settings can be specified, including:
  1. The products specified in the orders.
  2. The models in each bin (which products, the configuration).
  3. Which products are faulty products.
  4. What causes the second order to be announced.
  5. Which products are dropped (forthcoming).

- 
  There are also some features that have been specifically implemented for use in development, like spawning models in various reference frames.


- Here is a [sample development trial configuration file](https://github.com/usnistgov/ARIAC/blob/master/nist_gear/config/example_custom_config.yaml), which explains how you can modify all of the aforementioned settings.


## Improving real-time factor during development

- The real-time factor of a scenario is impacted by the number of models in the environment.
- For users experiencing low real-time factors, reducing the number of products that are in the scenario will help.
  - If you are focusing on grasping products from the bins, you can set `belt_population_cycles` to `0` to avoid spawning parts on the conveyor belt.
  - If you are focusing on grasping products from a particular bin, you can comment out the other bins listed in `models_over_bins` to temporarily not spawn them.
  - If you are focusing on grasping products from a particular shelf, you can comment out the other shelves listed in `models_over_shelves` to temporarily not spawn them.

-------------------------------------------------
- Wiki | [Home](../../README.md) | [Documentation](../documentation/documentation.md) | [Tutorials](../tutorials/tutorials.md) | [Qualifiers](../qualifiers/qualifier.md) | [Finals](../finals/finals.md)

-------------------------------------------------
