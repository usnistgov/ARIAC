# Details of Qualifier Scenarios

This page describes the scenarios that are used in [ARIAC 2020 qualification](qualifier.md).

The following applies to both scenarios:

* The trials have a time limit of 500 simulation seconds
  * Any unfulfilled shipments when the time limit is reached will not be scored
  * Subscribe to `/clock` to know the current simulation time
* The use of any "cheat" interfaces is forbidden, and will be blocked in the [automated evaluation setup](automated_evaluation.md)

# Part A
Part A is released for teams to practice with, and is also part of a teams qualification score.
There are 3 trial config files.

In the commands below, replace `<path/to/your/config/file.yaml>` with the path to your team's [environment configuration file](configuration_spec.md#markdown-header-competitor-configuration-file).

**Note:** By default simulation state logging is enabled in the trial config files. To disable it during development you can add `--state-logging=no` to the commands below.

## Part A Trial 1
![trial_config_1_at_start.png](https://bitbucket.org/repo/pB4bBb/images/2554684431-trial_config_1_at_start.png)


### Description

* There is a single order of one shipment:
    * The order will be updated at an inconvenient time.
    * The shipment will be scored against the updated order
* Faulty parts are present and should not be used to complete the order.

### How to run it

Run with your team's config file

```
rosrun osrf_gear gear.py -f $(catkin_find osrf_gear --share)/config/quals/qual_a_1.yaml <path/to/your/config/file.yaml> --development-mode --verbose
```

**Debugging**: visualize what perfect shipments would look like using the sample user config file

```
rosrun osrf_gear gear.py -f $(catkin_find osrf_gear --share)/config/quals/qual_a_1.yaml $(catkin_find osrf_gear --share)/config/quals/shipments/qual_a_1.yaml $(catkin_find osrf_gear --share)/config/sample_user_config.yaml --development-mode --verbose
```

## Part A Trial 2 
![trial_config_2_at_start.png](https://bitbucket.org/repo/pB4bBb/images/3724536791-trial_config_2_at_start.png)

### Description
* There is a single order of two shipments:
    * The shipments must be delivered on opposite AGVs.
* Products must be picked up off the conveyor
* Products must be handled by both arms to complete the order
* The gripper is faulty and will drop a product over each AGV

### How to run it

Run with your team's config file

```
rosrun osrf_gear gear.py -f $(catkin_find osrf_gear --share)/config/quals/qual_a_2.yaml <path/to/your/config/file.yaml> --development-mode --verbose
```

**Debugging**: visualize what perfect shipments would look like using the sample user config file

```
rosrun osrf_gear gear.py -f $(catkin_find osrf_gear --share)/config/quals/qual_a_2.yaml $(catkin_find osrf_gear --share)/config/quals/shipments/qual_a_2.yaml $(catkin_find osrf_gear --share)/config/sample_user_config.yaml --development-mode --verbose
```

## Part A Trial 3
![trial_config_3_at_start.png](https://bitbucket.org/repo/pB4bBb/images/4001782241-trial_config_3_at_start.png)

### Description

* There are two orders of one shipment each:
    * The second will interrupt the first at a convenient time.
    * The second has no time limit, but should be completed as fast as possible.
    * After the second order is complete, the first order is to be resumed.
* A product must be flipped to complete one of the orders.

### How to run it

Run with your team's config file

```
rosrun osrf_gear gear.py -f $(catkin_find osrf_gear --share)/config/quals/qual_a_3.yaml <path/to/your/config/file.yaml> --development-mode --verbose
```

**Debugging**: visualize what perfect shipments would look like using the sample user config file

```
rosrun osrf_gear gear.py -f $(catkin_find osrf_gear --share)/config/quals/qual_a_3.yaml $(catkin_find osrf_gear --share)/config/quals/shipments/qual_a_3.yaml $(catkin_find osrf_gear --share)/config/sample_user_config.yaml --development-mode --verbose
```


# Part B

Part B is not released for teams to practice with.
In it teams will be presented with a previously unseen scenario to test system autonomy.

Trial configs for Part B will have the following characteristics:

* Any or all [agility challenges](agility_challenges.md) may be present.
    * Faulty products
    * High-priority order interruption
    * Flipped products (only the `pulley_part` can be flipped)
    * Insufficiently many non-faulty products available
    * Updates to an existing order
    * Specific AGV destinations forcing products to be handled by both arms
    * Faulty gripper dropping products
    * Lost sensor communication
* There will be at most 2 orders, each made up of at most 2 shipments
* Same 6 storage bins
    * Some bins may be empty
    * At most 1 product type per bin
* Some products may be available only via the conveyor
* Same AGV and quality control sensor placement
* Only pulleys can be requested to be flipped
    * All products start in the bins un-flipped
