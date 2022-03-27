Wiki | [Home](../../README.md) | [Documentation](../documentation/documentation.md) | [Tutorials](../tutorials/tutorials.md) | [Qualifiers](../qualifiers/qualifier.md) | [Finals](../finals/finals.md)

---

- [Wiki | Documentation | Agility Challenges](#wiki--documentation--agility-challenges)
  - [Flipped Parts](#flipped-parts)
  - [Faulty Gripper](#faulty-gripper)
    - [After Grasping a Part from A Bin](#after-grasping-a-part-from-a-bin)
    - [When Placing a Part on an AGV](#when-placing-a-part-on-an-agv)
  - [Faulty Product](#faulty-product)
  - [Insufficient Products](#insufficient-products)
  - [Faulty Sensor](#faulty-sensor)
  - [High-priority Orders](#high-priority-orders)
  - [Robot Breakdown](#robot-breakdown)
  - [Human Obstacles](#human-obstacles)
  - [Gripper Change](#gripper-change)

# Wiki | Documentation | Agility Challenges

The ARIAC organizers wanted to be sure that the challenges that were captured within the ARIAC simulated environment were representative of the challenges faced by industry.

The following subsections provide a description of these challenges and examples of trial configuration files especially created to focus on the challenges.

During development, competitors will find it very useful to spawn parts directly on AGV trays, in assembly station briefcases, and on the assembly robot tray. The bash script [GearSpawner.sh](../../nist_gear/script/GearSpawner.sh) provides examples for spawning parts in different frames.

## Flipped Parts

- Trial sample: [flipped_parts_sample.yaml](../../nist_gear/config/trial_config/agility_challenges/flipped_parts_sample.yaml)

A part is presented to the robots in an orientation that is different than its desired final orientation. The robot needs to rotate the part around its x- or y-axis (or 'flip the part') before it is placed in the tray or in the briefcase.

The `pump` part (i.e., `assembly_pump_red`, `assembly_pump_blue`, and `assembly_pump_green`) is the only part in the environment designed to be flipped in this agility challenge. `pump` parts that need to be-reoriented are spawned in the environment with a `roll` value of 0. To know if a `pump` needs to be re-oriented, competitors have to check the `roll` value of each `pump` part in an order. If `roll` = ***pi*** then the part has to be re-oriented.

---

**Note**: On the topic `ariac/orders`, the orientation is in Quaternion.

---

Below is the snippet of `flipped_parts_sample.yaml`  which specifies that `assembly_pump_red` is required to be flipped.

```yaml
orders:
  order_0:
    priority: 1
    kitting_robot_health: 1
    assembly_robot_health: 1 
    # disable_robot: [kitting_robot, agv1, 1] # [kitting_robot, agv1, 1]
    announcement_condition: time
    announcement_condition_value: 0.0
    kitting:
      shipment_count: 1
      trays: [movable_tray_metal_shiny]
      agvs: [agv1]
      destinations: [as1]
      products:
        part_0:
          type: assembly_pump_red
          pose:
            xyz: [-0.1, -0.1, 0.0]
            rpy: ['pi', 0.0, 0.0]
```

`rostopic echo /ariac/orders -n 1` will display the order as follows:

```bash
order_id: "order_0"
kitting_shipments: 
  - 
    shipment_type: "order_0_kitting_shipment_0"
    agv: "agv1"
    assembly_station: "as1"
    movable_tray: 
      movable_tray_type: "movable_tray_metal_shiny"
      gripper: "gripper_tray"
      pose: 
        position: 
          x: 0.0
          y: 0.0
          z: 0.0
        orientation: 
          x: 0.0
          y: 0.0
          z: 0.0
          w: 1.0
    products: 
      - 
        type: "assembly_pump_red"
        gripper: "gripper_part"
        pose: 
          position: 
            x: -0.1
            y: -0.1
            z: 0.0
          orientation: 
            x: 1.0
            y: 0.0
            z: -0.0
            w: -1.03411553555e-13
assembly_shipments: []
  ---
```

If the trial starts with AGVs already located at assembly stations, then no part flipping will be required for competitors. It is not an easy task to flip a part with the assembly robot while this challenge is much more manageable with the kitting robot.

Although part flipping is not present during assembly, competitors may still need to rotate the parts around their z-axis to place them in slots in the briefcases.

## Faulty Gripper

- Trial sample: [faulty_gripper_over_bin_sample.yaml](../../nist_gear/config/trial_config/agility_challenges/faulty_gripper_over_bin_sample.yaml)
- Trial sample: [faulty_gripper_over_agv_sample.yaml](../../nist_gear/config/trial_config/agility_challenges/faulty_gripper_over_agv_sample.yaml)

This challenge emulates a gripper being faulty and it will occur only when there is a part in one of the robot's gripper. When there is a faulty gripper, as the robot is performing motions to pick up a part from a bin or place a part in a movable tray, the part will fall off the gripper. The robot needs to determine whether to re-grasp the part that just fell off or to get a new one from one of a part vessel.

---

**Note**: During this challenge the part may land in a bin or in the tray located on an AGV, or it may land on the floor. If the part lands on the floor, competitors will incur a penalty, however, since it is not competitors' fault that the part(s) landed on the floor, we will add the points back to the score, after the fact (manually).

---

There are two situations where this challenge can happen.

### After Grasping a Part from A Bin

The code snippet below, taken from `faulty_gripper_over_bin_sample.yaml` shows the decription of this challenge which is expected to occur when the **kitting** robot picks up a **red pump** from **bin #2**. The field `robot_type` can take one of the two values `kitting` (for the kitting robot) or `gantry` (for the gantry robot). In this example, the faulty gripper challenge will not be triggered after the gantry robot grasps a part from **bin #2**, it will happen only with the kitting robot.

```yaml
drops:
  drop_regions:
    shipping_box_0_impeding:
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

### When Placing a Part on an AGV

The code snippet below, taken from `faulty_gripper_over_agv_sample.yaml` shows the decription of this challenge which is expected to occur when the **gantry** robot is placing a **red pump** on **agv #1**.

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
      robot_type: gantry # or kitting
```

## Faulty Product

- Trial sample: [faulty_parts_sample.yaml](../../nist_gear/config/trial_config/agility_challenges/faulty_parts_sample.yaml)

The trial configuration file designates defective parts in part vessels through part IDs. Competitors are not aware of defective parts during trials. Once the robot places a part in a movable tray, a quality control sensor determines that the part is defective. The robot must dispose of the faulty part, as it does not count towards the trial score, and must get a new one from one of the part vessels.

The code snippet below shows how the faulty product challenge is described in a trial configuration file. Although faulty parts are the same for each competitor, it can happen that some competitors may encounter faulty parts during a trial and some other competitors may not encounter any faulty parts at all (if the robots are lucky enough to not pick up faulty parts).

```yaml
faulty_products:
  - assembly_battery_blue_2
  - assembly_regulator_red_1
```

**NOTE**: See the tutorial on [interacting with GEAR](../tutorials/gear_interface.md#Faulty-Products) for details on working with faulty products.


## Insufficient Products

- Trial sample: [insufficient_parts_sample.yaml](../../nist_gear/config/trial_config/agility_challenges/insufficient_parts_sample.yaml)

In this challenge the trial consists of kitting or assembly shipments that will need to be submitted as incomplete shipments. This is due to the environment not having enough non-faulty products to fill the shipments.

In the code snippet below, taken from `insufficient_parts_sample.yaml`,  the kitting shipment requires one red pump and one blue sensor. However, the workcell does not contain any blue sensor. This is done on purpose to see if competitors can submit incomplete shhipments.

```yaml
orders:
  order_0:
    priority: 1
    kitting_robot_health: 1
    assembly_robot_health: 1 
    announcement_condition: time
    announcement_condition_value: 0.0
    kitting:
      shipment_count: 1
      trays: [movable_tray_light_wood]
      agvs: [agv1]
      destinations: [as1]
      products:
        part_0:
          type: assembly_pump_red
          pose:
            xyz: [-0.1, -0.1, 0.0]
            rpy: [0.0, 0.0, 0.0]
        part_1:
          type: assembly_sensor_blue
          pose:
            xyz: [0.1, -0.1, 0.0]
            rpy: [0.0, 0.0, 'pi/2']

models_over_bins:
  bin2:
     models:
       assembly_pump_red:
         xyz_start: [0.2, 0.2, 0.0]
         xyz_end: [0.4, 0.4, 0.0]
         rpy: [0, 0, 0]
         num_models_x: 2
         num_models_y: 2
```

## Faulty Sensor

- Trial sample: [faulty_sensors_sample.yaml](../../nist_gear/config/trial_config/agility_challenges/faulty_sensors_sample.yaml)

For a finite period of time, all sensors in the factory stop working as to mimic a sensor blackout and communication with the sensors will be lost temporarily. At the start of the trial the sensors will publish data normally, and at a particular instance *all* sensors will *stop* publishing for a fixed period of time (10 to 100 simulation seconds). This applies to competitor-specified sensors and sensors that are present by default in the environment such as the quality control sensors.

Competitors' systems have to use an internal world model to continue to fill the order as usual during this time. Through the trial configuration file, ARIAC developers have control on the duration of this agility challenge.

Below is a snippet showing how this challenge is described in a trial configuration file. The challenge will start as soon as one product is placed in any briefcase or in any movable tray located on an AGV.

```yaml
sensor_blackout:
  product_count: 1
  duration: 20
```

**NOTE**: Re-connecting to some sensors during development will cause them to resume publishing data, but this functionality is blocked in the automated evaluation setup.

## High-priority Orders

- Trial sample: [hpo_wanted_products_sample.yaml](../../nist_gear/config/trial_config/agility_challenges/hpo_wanted_products_sample.yaml)
- Trial sample: [hpo_unwanted_products_sample.yaml](../../nist_gear/config/trial_config/agility_challenges/hpo_unwanted_products_sample.yaml)


Order announcements during trials are controlled in the trial configuration file with an announcement condition and an announcement value. The first order is  announced at the start of the competition with time and 0 for condition and value, respectively. An announcement condition can take two other separated values, namely `wanted_products` and `unwanted_products`. The value for each of these two conditions is an integer number ***n***, which is used  to control when a new order is announced. This agility challenge is mainly used to tests the ability of competitors' systems to put the previous order on hold, to quickly complete the new order, and to resume the previous order.

How `wanted_products` and `unwanted_products`  are useful depends on how much overlap there is between the previous order and the new one.

- When the condition is set to `wanted_products`, the previous order is interrupted when ***n*** products have been placed in the tray of the previous order that are also in the new order. In the code snippet below, `order_1` will be announced when 1 product placed during the completion of  `order_0` (not shown here) is needed in `order_1`.
  
```yaml
order_1:
  priority: 3
  kitting_robot_health: 1
  assembly_robot_health: 1 
  announcement_condition: wanted_products
  announcement_condition_value: 1
  kitting:
    shipment_count: 1
    trays: [movable_tray_metal_rusty]
    agvs: [agv2]
    destinations: [as2]
    products:
      part_0:
        type: assembly_pump_red
        pose:
          xyz: [-0.1, -0.1, 0]
          rpy: [0, 0, 0]
```

- When the condition is set to `unwanted_products`, the previous order is interrupted when ***n*** products not in the next order have been placed in the tray of the previous order. In the code snippet below, `order_1` will be announced when 1 product placed during the completion of  `order_0` (not shown here) is not needed in `order_1`.

```yaml
  order_1:
    priority: 3
    kitting_robot_health: 1
    assembly_robot_health: 1 
    announcement_condition: unwanted_products
    announcement_condition_value: 1
    kitting:
      shipment_count: 1
      trays: [movable_tray_metal_rusty]
      agvs: [agv2]
      destinations: [as2]
      products:
        part_0:
          type: assembly_pump_red
          pose:
            xyz: [-0.1, -0.1, 0]
            rpy: [0, 0, 0]
```

These conditions can make interesting scenarios, such as guaranteeing competitors have to remove parts or have to re-arrange parts in the tray of the previous order.

---
**Note**: High-priority orders will show up on the topic `/ariac/orders` as either `order_1` or as `order_0_update`. When an update to a previously assigned order is sent, it will be identifiable with the order ID such as `order_0_update_0`. Shipments will be evaluated against the updated order. Competitors should respond by filling the updated order as usual (submitting shipments named `order_0_shipment_0` still), instead of the original order.

## Robot Breakdown

- Trial sample: [robot_breakdown_sample.yaml](../../nist_gear/config/trial_config/agility_challenges/robot_breakdown_sample.yaml)

There will be situations where an event will disable the kitting robot or the assembly robot. This is described in trial configuration files with the following line:


```yaml
disable_robot: [<robot_type>, <location>, <number_of_product>]
```



- `<robot_type>`: The type of robot to disable. This tag can take the following values: `kitting_robot` or `assembly_robot`.
- `<location>`: The location where the event takes place. This tag can take the following values: `agv1`, `agv2`, `agv3`, `agv4`, `as1`, `as2`, `as3`, or `as4`.
- `<number_of_products>`: The number of products placed in an AGV or in a briefcase that will trigger this event.

Here are some examples of disabling a robot:

- `disable_robot: [kitting_robot, agv2, 1]` will disable the kitting robot after 1 product is placed in the movable tray located on agv2.
- `disable_robot: [kitting_robot, as1, 1]`  will disable the kitting robot after 1 product is placed in the briefcase located at assembly station `as1`". When this occurs, competitors will have to use only the assembly robot to do kitting.
- `disable_robot: [assembly_robot, agv2, 1]` disables the assembly robot after 1 product is placed on AGV2. This is to ensure that competitors using both robots for kitting will find themselves using only the kitting robot.

The following situation will never happen:

- `disable_robot: [assembly_robot, as2, 1]` will disable the assembly robot after 1 product is placed in briefcase located on assembly station 2. If this situation arises, competitors will not be able to complete the assembly shipment as the assembly robot is the only one capable of doing assembly.

**NOTE**: The ARIAC organizers will make sure that competitors do not find themselves locked after disabling a robot.

When these event occur, the disabled robot is not allowed to be used for any task for the given trial and the other robot will need to take over. When a robot is disabled, a new message will be published on the topic `/ariac/robot_health`.

```bash
$ rostopic echo /ariac/robot_health
---
kitting_robot_health: True
kitting_robot_health: True
---
```

The output above shows that both robots are enabled and can be used. If one of these flags is `False`, then the disabled robot should not be used. We are currently working on an approach to actually disable the robots in simulation by switching to static controllers. For now, competitors will need to read this topic to check the health status of the robots and not use any disabled robots.

**NOTE**: Once disabled, the robot will stay disabled for the whole order but maybe not for the whole trial. If a trial has another order, the disabled robot may be re-enabled for the new order. This is the purpose of the following fields for each order present in a trial:

```yaml
kitting_robot_health: 1
assembly_robot_health: 1 
```

## Human Obstacles

- Trial sample: [human_obstacles_sample.yaml](../../nist_gear/config/trial_config/agility_challenges/human_obstacles_sample.yaml)
  
Human obstacles consist of blocking some assembly stations for a certain amount of time, which is not communicated to competitors. While the humans are at the assembly stations, competitors should not perform assembly, as any collision with simulated humans will give a 0 for the trial.

In this challenge, there may be one or two simulated humans located in front of `as2` and/or `as4`. These humans will move away from those stations after a kitting shipment is submitted to `as2` or `as4`. However, these humans will not move away as soon as the AGV reaches the station. They will wait for some time before moving away. This wait time will vary between trials and it is not communicated to competitors.

The first challenge for competitors is to detect whether or not there are humans at `as2` and `as4`. The second challenge is to detect when these humans have moved away from the stations. Breakbeam sensors seem to be a good idea in this case.

Below is the snippet for describing this challenge. `location` must not be modified and will stay the same for all trials. This field specifies the **y** position of each human in the world frame. `start_time` is not currently being used and will be removed at a later date. `m0ve_time` indicates how long a human motion takes to complete and the value for this field will not change between trials. `wait_time` indicates how long the human waits after an AGV is submitted, before moving away from the assembly station. The value for this field will change between trials.

```yaml
aisle_layout:
  person_1: # located at as2
    location: 3
    start_time: 0.
    move_time: 5.
    wait_time: 15.
  person_2: # located at as4
    location: -3
    start_time: 16.
    move_time: 5.
    wait_time: 7.
```


## Gripper Change

This challenge affects only the gantry robot. The type of gripper attached to the gantry is either `gripper_part` or `gripper_tray`. The type of gripper is described in trial configuration files with:

```yaml
options:
  current_gripper_type: gripper_tray
```
or with:
```yaml
options:
  current_gripper_type: gripper_part
```

`gripper_part` can grasp any parts in the workcell but cannot grasp movable trays. `gripper_tray` can grasp any movable trays but not parts. There are situations when a gripper change is required. For instance, the environment can start with `gripper_part` attached to the gantry but the first order requires a kitting shipment. Since a kitting shipment needs a movable tray and only the gantry robot can grasp movable trays, the robot will first need to change gripper to grasp the movable tray. If assembly is requested in the same order, the gantry robot will have to change gripper again to grasp and maipulate parts.

To change gripper, the gantry robot has to position itself above the gripper changing station and call a ROS service to change gripper. Any location in the following boundaries will work:

```Text
-4.25 < x < -3.60
5.50 < y < 6.90
```

Once located above the gripper changing station, the call to the service `/ariac/gantry/arm/gripper/change {gripper_type}
` will change the gripper on the gantry. The current type of the gripper is published to the topic `/ariac/gantry/arm/gripper/type`.

---
**Note**: The kitting robot always have a `gripper_part` attached to the end effector and this cannot be changed. If for some reasons a movable tray located on an AGV needs to be discarded, then the gantry robot should be used.

---

Wiki | [Home](../../README.md) | [Documentation](../documentation/documentation.md) | [Tutorials](../tutorials/tutorials.md) | [Qualifiers](../qualifiers/qualifier.md) | [Finals](../finals/finals.md)
