Wiki | [Home](../../README.md) | [Documentation](../documentation/documentation.md) | [Tutorials](../tutorials/tutorials.md) | [Qualifiers](../qualifiers/qualifier.md) | [Finals](../finals/finals.md)

-------------------------------------------------

- [Wiki | Documentation | Agility Challenges](#wiki--documentation--agility-challenges)
  - [Spawner Script](#spawner-script)
  - [Part Re-orientation](#part-re-orientation)
    - [Kitting](#kitting)
    - [Assembly](#assembly)
  - [Faulty Gripper](#faulty-gripper)
  - [Faulty Product](#faulty-product)
  - [Insufficient Products](#insufficient-products)
  - [Faulty Sensor](#faulty-sensor)
  - [New Order](#new-order)
  - [New Task](#new-task)

# Wiki | Documentation | Agility Challenges

The ARIAC organizers wanted to be sure that the challenges that were captured within the ARIAC simulated environment were representative of the challenges faced by industry. As such, NIST reached out to industry. Each challenge was ranked with respect to its difficulty in representing it in Gazebo from 1 to 5 with 1 being the easiest to represent, as well as its importance to industry  from 1 to 5 with 1 being the most important. During the investigation phase, thirty-nine challenges were identified among which, six were selected to be focused on in ARIAC. These six challenges are listed in the Table below along with their respective ratings. All of these challenges have been represented in ARIAC at some point in the past, and many of them have been represented in all of the previous competitions.

<!-- We have provided a trial configuration file for each agility challenge (see right-most column). We have also provided a bash script which should help you spawn parts in the correct poses in the correct frame (briefcases, trays, and robot tray). This script will come in handy during debugging. -->



<!-- |      Challenges     | Difficulty rating | Importance rating | Trial configuration file                                                                             | Script to spawn products                                                                             |
|:-------------------:|:-----------------:|:-----------------:|------------------------------------------------------------------------------------------------------|------------------------------------------------------------------------------------------------------|
| Part re-orientation |         1         |         2         | [sample_part_reorientation.yaml](../../nist_gear/config/trial_config/sample_part_reorientation.yaml) | [sample_part_reorientation.sh](../../nist_gear/script/sample_part_reorientation.sh)   |
| Faulty gripper      |         1         |         4         | [sample_faulty_gripper.yaml](../../nist_gear/config/trial_config/sample_faulty_gripper.yaml)         | [sample_faulty_gripper.sh](../../nist_gear/script/sample_faulty_gripper.sh) |
| Faulty product      |         2         |         2         | [sample_kitting.yaml](../../nist_gear/config/trial_config/sample_faulty_product.yaml)         | [sample_part_reorientation.yaml](../../nist_gear/config/trial_config/sample_part_reorientation.yaml) |
| Faulty sensor       |         3         |         3         | [sample_faulty_sensor.yaml](../../nist_gear/config/trial_config/sample_faulty_sensor.yaml)           | [sample_part_reorientation.yaml](../../nist_gear/config/trial_config/sample_part_reorientation.yaml) |
| New order           |         3         |         5         | [sample_new_order.yaml](../../nist_gear/config/trial_config/sample_new_order.yaml)                   | [sample_part_reorientation.yaml](../../nist_gear/config/trial_config/sample_part_reorientation.yaml) |
| New task            |         4         |         5         | [sample_new_task.yaml](../../nist_gear/config/trial_config/sample_new_task.yaml)                     | [sample_part_reorientation.yaml](../../nist_gear/config/trial_config/sample_part_reorientation.yaml) | -->


|      Challenges     | Difficulty rating | Importance rating | Trial configuration file                                                                             |
|:-------------------:|:-----------------:|:-----------------:|------------------------------------------------------------------------------------------------------|
| Part re-orientation |         1         |         2         | [sample_part_reorientation.yaml](../../nist_gear/config/trial_config/sample_part_reorientation.yaml) |
| Faulty gripper      |         1         |         4         | [sample_faulty_gripper.yaml](../../nist_gear/config/trial_config/sample_faulty_gripper.yaml)         |
| Faulty product      |         2         |         2         | [sample_kitting.yaml](../../nist_gear/config/trial_config/sample_kitting.yaml)         |
| Insufficient product      |         2         |         2         | [sample_kitting.yaml](../../nist_gear/config/trial_config/sample_kitting.yaml)         |
| Faulty sensor       |         3         |         3         | [sample_faulty_sensor.yaml](../../nist_gear/config/trial_config/sample_faulty_sensor.yaml)           |
| New order           |         3         |         5         | [sample_new_order.yaml](../../nist_gear/config/trial_config/sample_new_order.yaml)                   |
| New task            |         4         |         5         | [sample_task_changeover.yaml](../../nist_gear/config/trial_config/sample_task_changeover.yaml)                     |

The following subsections provide a description of these challenges and examples of trial configuration files especially created to focus on the challenges.

## Spawner Script

During development, competitors will find it very useful to spawn parts directly on AGV trays, in assembly station briefcases, and on the assembly robot tray. The bash script [part_spawner.sh](../../nist_gear/script/part_spawner.sh) provides 3 examples for spawning a part in three different reference frames. 

**NOTE**: The model name (e.g., `-model assembly_pump_blue_12`) must be unique. If the environment already has a `assembly_pump_blue_12` then an error message will be issued if another `assembly_pump_blue_12` is spawned. Pick a large number for the part ID to have a better chance at spawning parts successfully.

## Part Re-orientation

**TASK**: Kitting

A part is presented to the robots in an orientation that is different than its desired final orientation. The robot needs to rotate the part around its x-axis (or 'flip the part') before it is placed in the tray or in the briefcase.

### Kitting

The `pump` part (i.e., `assembly_pump_red`, `assembly_pump_blue`, and `assembly_pump_green`) is the only part in the environment designed to be flipped in this agility challenge. `pump` parts that need to be-reoriented are spawned in the environment with a `roll` value of 0. To know if a `pump` needs to be re-oriented, competitors have to check the `roll` value of each `pump` part in an order. If `roll` = ***pi*** then the part has to be re-oriented.

**NOTE**: See documentation on [frame specifications](frame_specifications.md#flipped-products) for details on re-orienting products.

- Below is the snippet of the trial configuration file which specifies that `assembly_pump_blue` is required to be flipped.

  ```yaml
  orders:
    order_0:
      priority: 1
      kitting_robot_health: 1 # information on health for kitting robot
      assembly_robot_health: 1 # information on health for assembly robot
      announcement_condition: time
      announcement_condition_value: 0.0
      kitting:
        shipment_count: 1
        agvs: [agv2]
        destinations: [as1]
        products:
          part_0:
            type: assembly_pump_blue
            pose:
              xyz: [0.0, -0.2, 0]
              rpy: ['pi', 0, 0]
          part_1:
            type: assembly_regulator_red
            pose:
              xyz: [0.0, 0.2, 0]
              rpy: [0, 0, 'pi/2']
          part_2:
            type: assembly_sensor_blue
            pose:
              xyz: [0.0, 0.0, 0]
              rpy: [0, 0, 0]
  ```

- The trial configuration file above will announce the order below where `assembly_pump_blue` has to be re-oriented before being placed in the kit tray.

  ```bash
  order_id: "order_0"
  kitting_shipments: 
    - 
      shipment_type: "order_0_kitting_shipment_0"
      agv_id: "agv2"
      station_id: "as1"
      products: 
        - 
          type: "assembly_pump_blue"
          pose: 
            position: 
              x: 0.0
              y: -0.2
              z: 0.0
            orientation: 
              x: 1.0
              y: 0.0
              z: -0.0
              w: -1.03411553555e-13
        - 
          type: "assembly_regulator_red"
          pose: 
            position: 
              x: 0.0
              y: 0.2
              z: 0.0
            orientation: 
              x: 0.0
              y: 0.0
              z: 0.707106781185
              w: 0.707106781188
        - 
          type: "assembly_sensor_blue"
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
  assembly_shipments: []
  ---
  ```

### Assembly

If the trial starts with AGVs already located at assembly stations, then no part flipping will be required for competitors. It is not an easy task to flip a part with the assembly robot while this challenge is much more manageable with the kitting robot.

Although part flipping is not present during assembly, competitors will still need to re-orientate the parts to place them in slots in the briefcases.
<!-- 
  - The `battery` and the `pump` may need to be re-oriented around their $z$-axis (yaw). The two remaining parts (`sensor` and `regulator`) may be required to be re-oriented around the $x$-axis and $y$-axis before being inserted in their sockets. -->


## Faulty Gripper

**TASK**: Kitting

 In this challenge, as the robot is performing motions to place a part in a tray, the part drops out of the gripper and lands in the tray at a wrong location. The robot needs to determine whether to re-grasp the dropped part and replace it in the tray or to get a new one from one of the part vessels. The trial configuration files for this challenge describe the region in the workcell and the part type the robot must be holding to activate this challenge.

The code snippet below shows how this challenge is described in a trial configuration file. In this example, an `assembly_regulator_red` and an `assembly_sensor_blue` will drop from the gripper into the tray of `agv3`.

```yaml
drops:
  drop_regions:
    shipping_box_0_impeding:
      frame: agv3::kit_tray_3
      min:
        xyz: [-0.3, -0.3, 0.0]
      max:
        xyz: [0.3, 0.3, 0.5]
      destination:
        xyz: [0.3, 0.3, 0.05]
        rpy: [0, 0, 0.2]
      product_type_to_drop: assembly_regulator_red
    shipping_box_1_impeding:
      frame: agv3::kit_tray_3
      min:
        xyz: [-0.3, -0.3, 0.0]
      max:
        xyz: [0.3, 0.3, 0.5]
      destination:
        xyz: [0.3, 0.3, 0.05]
        rpy: [0, 0, 0.2]
      product_type_to_drop: assembly_sensor_blue
```

<!-- The gripper becomes faulty at various instances, e.g. when a product is retrieved from the storage bins, or when a product is being placed into a kit tray.
* Recovery could include retrieving the dropped product or fetching a new product.
* Sample trial:
  * [<b>sample_dropped_product.yaml</b>](https://github.com/usnistgov/ARIAC/blob/master/nist_gear/config/sample_dropped_product.yaml) -->

## Faulty Product

**TASK**: Kitting

The trial configuration file designates defective parts in part vessels through part IDs. Competitors are not aware of defective parts during trials. Once the robot places a part in a kit tray, a quality control sensor determines that the part is defective. The robot must dispose of the faulty part, as it does not count towards the trial score, and must get a new one from one the part vessels.

The code snippet below shows how the faulty product challenge is described in a trial configuration file. Competitors will be able to detect faulty products with the help of quality control sensors located above AGVs.

```yaml
faulty_products:
  - assembly_battery_blue_2
  - assembly_regulator_red_1
```

  <!-- * Faulty products should not be used to fulfill the orders.
  * Sample trial:
    * [<b>sample_faulty_products.yaml</b>](https://github.com/usnistgov/ARIAC/blob/master/nist_gear/config/sample_faulty_products.yaml) -->

**NOTE**: See the tutorial on [interacting with GEAR](../tutorials/gear_interface.md#Faulty-Products) for details on working with faulty products.

<!-- ## Insufficiently many products

 * Not enough non-faulty products are in the environment to fulfill all requested orders.
 * Teams must send the AGV with an incomplete kit.
 * Sample trial:
   * [<b>sample_not_enough_products.yaml</b>](https://github.com/usnistgov/ARIAC/blob/master/nist_gear/config/sample_not_enough_products.yaml) -->

## Insufficient Products

**TASK**: Kitting

In this challenge the trial consists of kitting shipments that will need to be submitted as incomplete kits. This is due to the environment not having enough non-faulty products to fill the shipments. 

## Faulty Sensor

**TASK**: Kitting and Assembly

For a finite period of time, all sensors in the factory stop working as to mimic a sensor blackout and communication with the sensors will be lost temporarily. At the start of the trial the sensors will be publishing data normally, and at a particular instance *all* sensors will *stop* publishing for a fixed period of time (10 to 100 simulation seconds). This applies to competitor-specified sensors and sensors that are present by default in the environment such as the quality control sensors.

Competitors' systems have to use an internal world model to continue to fill the order as usual during this time. Through the trial configuration file, ARIAC developers have control on the duration of this agility challenge.

**NOTE**: Re-connecting to some sensors during development will cause them to resume publishing data, but this functionality is blocked in the automated evaluation setup.

## New Order

**TASK**: Kitting

Order announcements during trials are controlled in the trial configuration file with an announcement condition and an announcement value. The first order is  announced at the start of the competition with time and 0 for condition and value, respectively. An announcement condition can take two other separated values, namely `wanted_products` and `unwanted_products`. The value for each of these two conditions is an integer number ***n***, which is used  to control when a new order is announced. This agility challenge is mainly used in the High-priority Kit Change scenarios to tests the ability of competitors' systems to put the previous order on hold, to quickly complete the new order, and to resume the previous order.

How `wanted_products` and `unwanted_products`  are useful depends on how much overlap there is between the previous order and the new one.

- When the condition is set to `wanted_products`, the previous order is interrupted when ***n*** products have been placed in the tray of the previous order that are also in the new order.
  
- When the condition is set to `unwanted_products`, the previous order is interrupted when ***n*** products not in the next order have been placed in the tray of the previous order.

These conditions can make interesting scenarios, such as guaranteeing competitors have to remove parts or have to re-arrange parts in the tray of the previous order.



## New Task

**TASK**: Kitting and Assembly

There will be situations where an event will disable the kitting robot or the assembly robot. This is described in trial configuration files with the following line:

```yaml
disable_robot: [<robot_type>, <location>, <number_of_product>]
```

- `<robot_type>`: The type of robot to disable. This tag can take the following values: `kitting_robot` or `assembly_robot`.
- `<location>`: The location where the event takes place. This tag can take the following values: `agv1`, `agv2`, `agv3`, `agv4`, `as1`, `as2`, `as3`, or `as4`.
- `<number_of_products>`: The number of products placed in an AGV or in a briefcase that will trigger this event.

Here are some examples of disabling a robot:

- `disable_robot: [kitting_robot, agv2, 1]` will disable the kitting robot after 1 product is placed in agv2.
- `disable_robot: [kitting_robot, as1, 1]`  will disable the kitting robot after 1 product is placed in the briefcase located at assembly station `as1`". When this occurs, competitors will have to use the assembly robot to finish the kit that the kitting robot has started.
- `disable_robot: [assembly_robot, agv2, 1]` disables the assembly robot after 1 product is placed on AGV2. This is to ensure that competitors using both robots for kitting will find themselves using only the kitting robot.

The following situation will never happen:

- `disable_robot: [assembly_robot, as2, 1]` will disable the assembly robot after 1 product is placed in briefcase located on assembly station 2. If this situation arises, competitors will not be able to complete the assembly shipment as the assembly robot is the only one capable of doing assembly.

**NOTE**: The ARIAC organizers will make sure that competitors do not find themselves locked after disabling a robot.

When these event occur, the disabled robot is not allowed to be used for any task for the given trial and the other robot will need to take over. When a robot is disabled, a new message will be published on the topic `/ariac/robot_health`.


```bash
$ rostopic echo /ariac/robot_health
---
kitting_robot_enabled: True
assembly_robot_enabled: True
---
```

The output above shows that both robots are enabled and can be used. If one of these flags is `False`, then the disabled robot should not be used. We are currently working on an approach to actually disable the robots in simulation. For now, competitors will need to read this topic to check the health status of the robots and not use any disabled robots.

**NOTE**: Once disabled, the robot will stay disabled for the whole order but maybe not for the whole trial. If a trial has another order, the disabled robot may be re-enabled for the new order. Do not forget to subscribe to the topic `/ariac/robot_health`.

<!-- * An update to a previously assigned order is sent, identifiable with the order ID such as "order_0_update_0".
* Shipments will be evaluated against the updated order.
* Teams should respond by filling the updated order as usual (submitting shipments named "order_0_shipment_0" still), instead of the original order.
* Sample trial:
  * [<b>sample_order_update.yaml</b>](https://github.com/usnistgov/ARIAC/blob/master/nist_gear/config/sample_order_update.yaml) -->


<!-- * Sample trial:
  * [<b>sample_sensor_blackout.yaml</b>](https://github.com/usnistgov/ARIAC/blob/master/nist_gear/config/sample_sensor_blackout.yaml) -->

<!-- ## In-process order interruption

* A second order is announced part-way into the completion of the first order.
* Kits from both orders can be submitted after this time, but the second order is higher priority and for maximum points it should be completed as fast as possible.
* Sample trial:
  * [<b>sample_interruption.yaml</b>](https://github.com/usnistgov/ARIAC/blob/master/nist_gear/config/sample_interruption.yaml) -->

-------------------------------------------------
Wiki | [Home](../../README.md) | [Documentation](../documentation/documentation.md) | [Tutorials](../tutorials/tutorials.md) | [Qualifiers](../qualifiers/qualifier.md) | [Finals](../finals/finals.md)
