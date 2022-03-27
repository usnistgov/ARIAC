Wiki | [Home](../../README.md) | [Documentation](../documentation/documentation.md) | [Tutorials](../tutorials/tutorials.md) | [Qualifiers](../qualifiers/qualifier.md) | [Finals](../finals/finals.md)

---

- [Wiki | Documentation | YAML Configuration Files](#wiki--documentation--yaml-configuration-files)
  - [User Configuration File](#user-configuration-file)
    - [How to Add Sensors](#how-to-add-sensors)
    - [How to Visualize Sensor Views](#how-to-visualize-sensor-views)
    - [How to Read Sensor Data](#how-to-read-sensor-data)
  - [Trial Configuration File](#trial-configuration-file)
    - [Custom competition configuration files](#custom-competition-configuration-files)
      - [`options` Field](#options-field)
      - [`time_limit` Field](#time_limit-field)
      - [`agv_infos` Field](#agv_infos-field)
      - [`orders` Field](#orders-field)
        - [`kitting` and `assembly` Fields](#kitting-and-assembly-fields)
      - [`models_over_bins` Field](#models_over_bins-field)
      - [`models_over_stations` Field](#models_over_stations-field)
      - [`belt_models` Field](#belt_models-field)
      - [`faulty_products` Field](#faulty_products-field)
      - [`drops` Field](#drops-field)
      - [`sensor_blackout` Field](#sensor_blackout-field)
  - [Improving real-time factor during development](#improving-real-time-factor-during-development)

# Wiki | Documentation | YAML Configuration Files

This page describes how to configure the simulated workcell environment by selecting and placing sensors. Moreover, during development, competitors will find it useful to create new or update existing trial configuration files in order to configure the behavior and the challenges during trials. Therefore, this page consists of a section describing each field of a trial configuration file. Trial and user configuration files are [YAML](http://yaml.org/) formatted files.

```diff
- Competitors are not allowed to read trial configuration files and user configuration files directly. 
- Everything has to be done through the API.
```

- A trial configuration file details specifics of a particular trial of the competition. In a trial configuration file, ARIAC organizers can describe orders, shipments, agility challenges, and the location of parts in the environment. These files are used to test competitors' systems during the qualifier and final rounds. Configuration files are located in the [nist_gear/config](../../nist_gear/config) directory.
- A user configuration file is a file created by each competitor which describes the type and the location of each sensor in the workcell. During the qualifier and final rounds, one single user configuration file will be used for all the trials. User configuration files should be placed in the [nist_gear/config/user_config](../../nist_gear/config/user_config) directory.

Both these files are passed to the `sample_environment.launch` file. The example below uses the trial config file `kitting_sample.yaml` and the sensor config file `sample_user_config.yaml`.

```xml
<node name="ariac_sim" pkg="nist_gear" type="gear.py"
        args="--development-mode
          $(arg verbose_args)
          $(arg state_logging_args)
          $(arg gui_args)
          $(arg load_moveit_args)
          $(arg load_gantry_moveit_args)
          $(arg load_kitting_moveit_args)
          $(arg fill_demo_shipment_args)
          --visualize-sensor-views
          -f $(find nist_gear)/config/trial_config/manufacturing_tasks/only_kitting/kitting_sample.yaml
          $(find nist_gear)/config/user_config/sample_user_config.yaml
          " required="true" output="screen" />
```

<!-- 
Both of these files are passed to an invocation of `gear.py` (located in `nist_gear/script/` which parses the data and launches the appropriately configured simulation. The general format of the invocation is:


```
rosrun nist_gear gear.py -f <trial_config_file> <competitor_config_file>
```
 -->

## User Configuration File

Competitors are allowed to select the quantity, type, and location of sensors. Sensors can only be placed in static locations, they cannot be attached to the robots (unless done by the organizers themselves) or otherwise be moved around the environment. Choices of sensors must be written using the YAML syntax. A sensor's position and orientation is specified in global coordinates (world frame) using an XYZ vector and Euler angles (roll, pitch, yaw).

### How to Add Sensors

The configuration YAML file contains a list of sensors denoted by the `sensors:` tag. Each sensor should have a **unique** name followed by the type of sensor and the sensor's position and orientation. Available sensor types include:

- break_beam
- proximity
- logical_camera
- laser_profiler
- depth_camera
- rgbd_camera
- cameras mounted on the gantry robot

An example of a user configuration file is shown below. Note that `gantry_bin_camera` and `gantry_tray_camera` are not within `sensors`. These two entries are required and must be set to `true` or `false`.

```yaml
sensors:
  logical_camera_0:
    type: logical_camera
    pose:
      xyz: [-2.515033, 2.925223, 1.82]
      rpy: [-1.570796, 1.570796, 0]
  breakbeam_0:
    type: break_beam
    pose:
      xyz: [-0.557152, 4.485425, 0.879306]
      rpy: [0, 0, -1.557900] 
gantry_bin_camera:
  enable: true
  
gantry_tray_camera:
  enable: true
```

### How to Visualize Sensor Views

By default, the view of the sensors in the Gazebo simulation will not be displayed. Enabling the sensor visualization may be useful while you are deciding where to place sensors in the world. Competitors can enable sensor visualization by adding `--visualize-sensor-views` in the launch file (as seen in `sample_environment.launch` file above).

### How to Read Sensor Data

This is covered by the [sensor interface tutorial](../tutorials/sensor_interface.md).

## Trial Configuration File

Each trial of the competition is specified using a separate configuration file. A number of example trials are provided with the ARIAC software in the `nist_gear/config/trial_config` directory. Each config file has a description of its behavior at the top of the file, e.g.:

```yaml
#Number of orders: 1
#Tasks: kitting
#Number of shipments: 1
#Insufficiently many products: No
#Part re-orientation: No
#Dropped products: Yes
#In-process order interruption: No
#Faulty products: No
#Sensor blackout: No
#Time limit: No
```

These config files can be used to practice with the [various agility challenges](agility_challenges.md).

### Custom competition configuration files

This section describes the different fields that can be found in trial configuration files. Some of these fields are optional and work only in development mode. Various settings can be specified, including:

- The number of orders.
- The number of shipments per order.
- The type of movable trays for kitting shipments.
- The product types and pose in each shipment.
- Location to build the shipments (AGVs or briefcases).
- The location of AGVs in the workcell.
- The models in each bin, in briefcases, and on AGVs (type and pose).
- Agility challenges:
  - Which products are faulty products.
  - What causes the high-priority order to be announced.
  - Which product type should be used for the faulty gripper challenge,
  - Which robot to disable and when.
  - When to trigger faulty sensors and how long it lasts.
  - Which parts need to be flipped.

An example of a trial configuration file is given below. This file consists of all the possible fields and are explained a further down in this section.

<!-- - Here is a [sample development trial configuration file](https://github.com/usnistgov/ARIAC/blob/master/nist_gear/config/example_custom_config.yaml), which explains how you can modify all of the aforementioned settings. -->

```yaml
options:
  insert_models_over_bins: true
  insert_models_over_stations: true
  belt_population_cycles: 15
  # mandatory: gripper_tray or gripper_part
  current_gripper_type: gripper_tray
time_limit: -1

# The table_tray_infos is not mandatory
# The different options for tray_model are:
#  -movable_tray_metal_shiny
#  -movable_tray_metal_rusty
#  -movable_tray_dark_wood
#  -movable_tray_light_wood
# quantity can be: 1, 2, or 3
table_tray_infos:
  table_1: 
    tray_model: movable_tray_metal_shiny
    quantity: 1
  table_2: 
    tray_model: movable_tray_metal_rusty
    quantity: 1

agv_infos:
  agv1:
    location: ks1
  agv2:
    location: ks2
  agv3:
    location: ks3
  agv4:
    location: as3   
    products:
      part_0:
        type: assembly_pump_blue
        pose:
          xyz: [0.15, -0.1, 0]
          rpy: [0, 0, 0]
      part_1:
        type: assembly_battery_green
        pose:
          xyz: [0.0, 0, 0]
          rpy: [0, 0, 0]
orders:
  order_0:
    priority: 1
    kitting_robot_health: 1 
    assembly_robot_health: 1 

    #   disable_robot: [kitting_robot, as3, 1] # [gantry_robot, agv1, 1]
    announcement_condition: time
    announcement_condition_value: 0.0
    kitting:
      shipment_count: 1
      trays: [movable_tray_metal_rusty]
      agvs: [agv2]
      destinations: [as1]
      products:
        part_0:
          type: assembly_battery_blue
          pose:
            xyz: [0.1, 0.1, 0]
            rpy: [0, 0, 0]
        part_1:
          type: assembly_regulator_red
          pose:
            xyz: [0.15, 0.0, 0]
            rpy: [0, 0, 'pi/4']
    assembly:
      shipment_count: 1
      stations: [as3]
      products:
        part_0:
          type: assembly_pump_blue
          pose:
            xyz: [-0.032465, 0.174845, 0.15]
            rpy: [0, 0, 0]
        part_1:
          type: assembly_battery_green
          pose:
            xyz: [0.032085, -0.152835, 0.25]
            rpy: [0, 0, 0]
  order_1:
    priority: 3 # default is 1 if omitted
    kitting_robot_health: 1 # information on health for kitting robot
    assembly_robot_health: 1 # information on health for assembly robot
    announcement_condition: unwanted_products
    announcement_condition_value: 1
    #   announcement_condition: kitting_submission
    #   announcement_condition_value: [agv1, as1]

    #   announcement_condition: assembly_submission
    #   announcement_condition_value: [as1]

    #   announcement_condition: wanted_products
    #   announcement_condition_value: 1
    kitting:
      shipment_count: 1
      trays: [movable_tray_metal_shiny]
      agvs: [agv3]
      destinations: [as4]
      products:
        part_0:
          type: assembly_sensor_blue
          pose:
            xyz: [0.1, 0.1, 0]
            rpy: ['pi/2', 0, 0]

        
models_over_bins:
   bin1:
     models:
       assembly_battery_blue:
         xyz_start: [0.2, 0.2, 0.0]
         xyz_end: [0.4, 0.4, 0.0]
         rpy: [0, 0, 'pi/4']
         num_models_x: 2
         num_models_y: 2
   bin4:
     models:
       assembly_battery_red:
         xyz_start: [0.2, 0.2, 0.0]
         xyz_end: [0.4, 0.4, 0.0]
         rpy: [0, 0, 'pi/4']
         num_models_x: 2
         num_models_y: 2

models_over_stations:
  as4:
    models:
      assembly_battery_green:
        xyz: [-0.032465, 0.174845, 0.15]
        rpy: [0, 0, 0]

belt_models:
  assembly_pump_blue:
    1.0:
      pose:
        xyz: [0.0, 4.3, 0.92]
        rpy: [0, 0, 'pi/2']
  assembly_regulator_red:
    2.0:
      pose:
        xyz: [0.0, 4.3, 0.92]
        rpy: [0, 0, 'pi/2']

faulty_products:
  - assembly_battery_blue_2
  - assembly_regulator_red_1

drops:
  drop_regions:
    shipping_box_0_impeding:
      frame: agv1::kit_tray_1
      min:
        xyz: [-0.3, -0.3, 0.0]
      max:
        xyz: [0.3, 0.3, 0.5]
      destination:
        xyz: [0.2, 0.2, 0.05]
        rpy: [0, 0, 0.2]
      product_type_to_drop: assembly_pump_red
      robot_type: kitting # or gantry
    shipping_box_1_impeding:
      frame: bin2
      min:
        xyz: [-0.3, -0.3, 1.0]
      max:
        xyz: [0.3, 0.3, 3.0]
      destination:
        xyz: [0.1, 0.1, 0.78]
        rpy: [0, 0, 0.2]
      product_type_to_drop: assembly_pump_red
      robot_type: kitting # or gantry

sensor_blackout:
  product_count: 1 #sensor blackout triggered after second product is placed
  duration: 50
# empty line at the end


```

#### `options` Field

```yaml
options:
  insert_models_over_bins: true
  insert_models_over_stations: true
  belt_population_cycles: 15
  time_limit: -1
```

The `options` field is used to set parameters which will orchestrate many things in the trial.

- `insert_models_over_bins` allows models in bins to spawn in the workcell.
- `insert_models_over_stations` allows to start the trial with briefcases partially filled with some parts. Competitors will need to complete the briefcases with missing parts for the ventilators.
- `belt_population_cycles` informs how many parts will spawn on the belt. In this example, 15 parts. This option is linked with the field `belt_models`. Setting the value for this field to 0 will deactivate the belt for this trial.
- `current_gripper_type` is mandatory. This field takes one the two values `gripper_part` or `gripper_tray`. This field defines the type of gripper mounted on the gantry when the simulation starts.

#### `time_limit` Field

`time_limit` dictates how long competitors have to complete the trial.

* `-1`: No time limit.

- `500`: Time limit used during qualifiers and finals (in simulation seconds unit).

#### `agv_infos` Field

```yaml
agv_infos:
  agv1:
    location: ks1
  agv2:
    location: ks2
  agv3:
    location: ks3
  agv4:
    location: as3   
    products:
      part_0:
        type: assembly_pump_blue
        pose:
          xyz: [0.15, -0.1, 0]
          rpy: [0, 0, 0]
      part_1:
        type: assembly_battery_green
        pose:
          xyz: [0.0, 0, 0]
          rpy: [0, 0, 0]
```

The `agv_infos` field is **mandatory**. It describes the location of each AGV (`agv1`, `agv2`, `agv3`, and `agv4`) when a trial starts. Each AGV has 3 different locations (as seen in the Table below) where `ks` stands for kitting station and `as` stands for assembly station.


|       | `agv1` | `agv2` | `agv3` | `agv4` |
| ------- | -------- | -------- | -------- | -------- |
| `ks1` | x      |        |        |        |
| `ks2` |        | x      |        |        |
| `ks3` |        |        | x      |        |
| `ks4` |        |        |        | x      |
| `as1` | x      | x      |        |        |
| `as2` | x      | x      |        |        |
| `as3` |        |        | x      | x      |
| `as4` |        |        | x      | x      |

The `products` field is optional and describes products located in AGV trays. When the `products` field is used, each part's type (`type`) and pose `pose` (`xyz` and `rpy`) must be present. The pose is referenced in the tray frame. Note that each part must be represented by a unique id (`part_0` and `part_1` in this example).

**Note**: When an AGV is already located at an assembly station when simulation starts, this AGV will not have a movable tray on its back, this is expected. The AGV in this case is only used to provide parts for assembly.

#### `orders` Field

```yaml
orders:
  order_0:
    priority: 1
    kitting_robot_health: 1 
    assembly_robot_health: 1 

    #   disable_robot: [kitting_robot, as3, 1] # [gantry_robot, agv1, 1]
    announcement_condition: time
    announcement_condition_value: 0.0
    kitting:
      shipment_count: 1
      trays: [movable_tray_metal_rusty]
      agvs: [agv2]
      destinations: [as1]
      products:
        part_0:
          type: assembly_battery_blue
          pose:
            xyz: [0.1, 0.1, 0]
            rpy: [0, 0, 0]
        part_1:
          type: assembly_regulator_red
          pose:
            xyz: [0.15, 0.0, 0]
            rpy: [0, 0, 'pi/4']
    assembly:
      shipment_count: 1
      stations: [as3]
      products:
        part_0:
          type: assembly_pump_blue
          pose:
            xyz: [-0.032465, 0.174845, 0.15]
            rpy: [0, 0, 0]
        part_1:
          type: assembly_battery_green
          pose:
            xyz: [0.032085, -0.152835, 0.25]
            rpy: [0, 0, 0]
  order_1:
    priority: 3 # default is 1 if omitted
    kitting_robot_health: 1 # information on health for kitting robot
    assembly_robot_health: 1 # information on health for assembly robot
    announcement_condition: unwanted_products
    announcement_condition_value: 1
    #   announcement_condition: kitting_submission
    #   announcement_condition_value: [agv1, as1]

    #   announcement_condition: assembly_submission
    #   announcement_condition_value: [as1]

    #   announcement_condition: wanted_products
    #   announcement_condition_value: 1
    kitting:
      shipment_count: 1
      trays: [movable_tray_metal_shiny]
      agvs: [agv3]
      destinations: [as4]
      products:
        part_0:
          type: assembly_sensor_blue
          pose:
            xyz: [0.1, 0.1, 0]
            rpy: ['pi/2', 0, 0]
```

The field `orders` is **mandatory** and consists of at least one order (`order_0`). Each order must have a unique id, with the first order being `order_0` and the second order being `order_1`. Situations where the current order must be updated will be signaled with the `_update` suffix (e.g., `order_0_update`).

- `priority`: This field informs on the priority of the order. If this field is not provided then the priority of the order is 1. In the example, we could have omitted the line `priority: 1`. However, if an order is of high priority, such as `order_1` in the example, we **must** set the `priority` field, otherwise it would be set to 1 by default.
- `kitting_robot_health` and `assembly_robot_health` describe whether the kitting and assembly robots are enabled (`1`) or disabled (`0`). These two fields are optional and if not used, their values are set to `1`. In this example, when the first order is announced, both robots are enabled. In `order_1` both robots are also enabled.
- `disable_robot`: This field is optional but when provided it will disable one of the two robots when a certain condition is reached. In this example, the kitting robot will be disabled as soon as one part is placed in the briefcase located at assembly station 3. A new message will be published on the topic `ariac/robot_health`, setting the status of the kitting robot to `False`.
- `announcement_condition` and `announcement_condition_value` informs GEAR when to announce `order_0`. In this example the order will be announced at `time = 0.0`, that is, as soon as the service `ariac/start_competition` is called. There are 4 other possibilities for `announcement_condition`, they are `wanted_products`, `unwanted_products`, `kitting_submission`, and `kitting_submission` (see [Competition Specifications](competition_specifications.md) for more information on these  fields).

##### `kitting` and `assembly` Fields

The `kitting` and `assembly` fields are both shipments. In this example, `order_0` consists of two shipments, a `kitting` shipment which needs to be submitted by delivering an AGV to an assembly station and an `assembly` shipment which needs to be submitted (nothing moves in the environment when an `assembly` shipment is submitted).

- `kitting` field.

  - All the subfields in the `kitting` field are **mandatory**. `shipment_count` specifies how many times a `kitting` shipment must be submitted, `trays` informs on the type of movable tray needed for this shipment, `agvs` informs on the AGV to use for this shipment, and `destinations` informs on the assembly station that the shipment must be delivered. `agvs` and `destinations` must have the same number of items  and there is a one to one match. For instance, if this shipment was to be submitted twice (`shipment_count: 2`), then we would have two entries for `agvs` and two entries for `destinations`. The first entry for `agvs` pairs with the first entry in `destinations` and the second entry in `agvs` pairs with the second entry in `destinations`.  **NOTE**: `destinations` must be one of the two assembly stations for the given AGV (see Table above). An error will be raised if a non-reachable assembly station is provided for the given AGV.
  - `products` describes the products' types and poses that need to be placed in the tray reference frame.

    ```yaml
      kitting:
        shipment_count: 1
        trays: [movable_tray_metal_shiny]
        agvs: [agv2]
        destinations: [as1]
        products:
            part_0:
              type: assembly_battery_blue
              pose:
                xyz: [0.1, 0.1, 0]
                rpy: [0, 0, 0]
            part_1:
              type: assembly_regulator_red
              pose:
                xyz: [0.15, 0.0, 0]
                rpy: [0, 0, 'pi/4']
    ```
- `assembly` field.

  - The subfields for this field are also **mandatory** and have the same meanings as the subfields for the `kitting` field. The `stations` field informs competitors where the assembly must be performed. The number of entries for `stations` has to match the value for the field `shipment_count`.

    ```yaml
        assembly:
        shipment_count: 1
        stations: [as3]
        products:
            part_0:
            type: assembly_pump_blue
            pose:
                xyz: [-0.032465, 0.174845, 0.15]
                rpy: [0, 0, 0]
            part_1:
            type: assembly_battery_green
            pose:
                xyz: [0.032085, -0.152835, 0.25]
                rpy: [0, 0, 0]
    ```

#### `models_over_bins` Field

```yaml
models_over_bins:
   bin1:
     models:
       assembly_battery_blue:
         xyz_start: [0.2, 0.2, 0.0]
         xyz_end: [0.4, 0.4, 0.0]
         rpy: [0, 0, 'pi/4']
         num_models_x: 2
         num_models_y: 2
   bin4:
     models:
       assembly_battery_red:
         xyz_start: [0.2, 0.2, 0.0]
         xyz_end: [0.4, 0.4, 0.0]
         rpy: [0, 0, 'pi/4']
         num_models_x: 2
         num_models_y: 2
```

This field is optional (since parts can spawn on the conveyor belt only) and informs the type and the configuration of products in bins. All the subfields for this field are **mandatory**. The `models` field describes the products and their configuration within a matrix. In this example, there are 4 products in each bin (`num_models_x`$\times$`num_models_y`). While the position of each product is not explicitly specified, their orientation is set with the subfield `rpy`. Each bin has only one type of parts and there cannot be a mix of part types in the same bin.

#### `models_over_stations` Field

```yaml
models_over_stations:
  as4:
    models:
      assembly_battery_green:
        xyz: [-0.032465, 0.174845, 0.15]
        rpy: [0, 0, 0]
```

This field is optional. When used, all the subfields for this field are **mandatory**. The first subfield (`as4`) informs GEAR to place parts in the briefcase located at assembly station 4. Competitors will only need to place the missing parts to complete the ventilators. `models` informs on the part types and poses inside the briefcase.

#### `belt_models` Field

```yaml
belt_models:
  assembly_pump_blue:
    1.0:
      pose:
        xyz: [0.0, 4.3, 0.92]
        rpy: [0, 0, 'pi/2']
  assembly_regulator_red:
    2.0:
      pose:
        xyz: [0.0, 4.3, 0.92]
        rpy: [0, 0, 'pi/2']
```

For this field to be used by GEAR, the option `belt_population_cycles` must be set to non-zero. In our example, `belt_population_cycles: 15` will spawn 15 parts (30 parts in total) for each part type defined in `belt_models`. In the code snippet provided above, the belt will spawn two different part types. The firs part (`assembly_pump_blue`) will spawn 1 s after the competition starts, 2 s after the first part spawns, an `assembly_regulator_red` will spawn. Another `assembly_pump_blue` will spawn 1 s after `assembly_regulator_red` is spawned and so forth. In trials, the belt will only spawn one part type and not two, as shown in the example.

#### `faulty_products` Field

```yaml
faulty_products:
  - assembly_battery_blue_2
  - assembly_regulator_red_1
```

This field informs GEAR which products in the environment are faulty and is used to set the [Faulty Product challenge](agility_challenges.md#faulty-product). The subfields consist of parts with the same ids as parts found in the simulation environment.

#### `drops` Field

```yaml
drops:
  drop_regions:
    shipping_box_0_impeding:
      frame: agv1::kit_tray_1
      min:
        xyz: [-0.3, -0.3, 0.0]
      max:
        xyz: [0.3, 0.3, 0.5]
      destination:
        xyz: [0.2, 0.2, 0.05]
        rpy: [0, 0, 0.2]
      product_type_to_drop: assembly_pump_red
      robot_type: kitting # or gantry
    shipping_box_1_impeding:
      frame: bin2
      min:
        xyz: [-0.3, -0.3, 1.0]
      max:
        xyz: [0.3, 0.3, 3.0]
      destination:
        xyz: [0.1, 0.1, 0.78]
        rpy: [0, 0, 0.2]
      product_type_to_drop: assembly_pump_red
      robot_type: kitting # or gantry
```

This field is used for the [Faulty Gripper challenge](agility_challenges.md#faulty-gripper) and informs which part types to drop and where in the tray frame. This can be easily customized to drop parts anywhere in the tray or even on the floor. In `shipping_box_0_impeding`, a part will drop from the kitting robot's gripper (specified with `robot_type: kitting`) when the robot is placing a red pump (specified with `product_type_to_drop: assembly_pump_red`) on AGV1 (specified with `frame: agv1::kit_tray_1`). When the part is located within a virtual cube (specified with `min` and `max`), it will be dropped (teleported) to `destination`.

The second example described in `shipping_box_1_impeding` will force a part drop after a kitting robot picks up a red pump from `bin2`.

#### `sensor_blackout` Field

```yaml
sensor_blackout:
  product_count: 1 #sensor blackout triggered after second product is placed
  duration: 50
```

This field is used to trigger the [Faulty Sensor challenge](agility_challenges.md#faulty-sensor). The condition to start the challenge is described with `product_count` and `duration` describes how long the sensors stay disabled (in simulation seconds). In this example, the challenge starts as soon as one product is placed on any AGV or in any briefcase.

## Improving real-time factor during development

- The real-time factor of a scenario is impacted by the number of models in the environment.
- For users experiencing low real-time factors, reducing the number of products that are in the scenario will help.
  - If you are focusing on grasping products from the bins, you can set `belt_population_cycles` to `0` to avoid spawning parts on the conveyor belt.
  - If you are focusing on grasping products from a particular bin, you can comment out the other bins listed in `models_over_bins` to temporarily not spawn them.
  - If you are focusing on assembly, comment out the field `table_tray_infos` to not have any movable tray in the environment. Each movable tray has a contact sensor, which is constantly updating in the background. Not having movable trays in the environment should help with the real-time factor.

---

Wiki | [Home](../../README.md) | [Documentation](../documentation/documentation.md) | [Tutorials](../tutorials/tutorials.md) | [Qualifiers](../qualifiers/qualifier.md) | [Finals](../finals/finals.md)
