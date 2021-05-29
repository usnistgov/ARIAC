Wiki | [Home](../../README.md) | [Documentation](../documentation/documentation.md) | [Tutorials](../tutorials/tutorials.md) | [Qualifiers](../qualifiers/qualifier.md) | [Finals](../finals/finals.md)

-------------------------------------------------


- [Wiki | Finals](#wiki--finals)
  - [Important Dates](#important-dates)
  - [Evaluation](#evaluation)
  - [Finals Practice](#finals-practice)
  - [Finals Specifications](#finals-specifications)
    - [Competition management](#competition-management)
    - [Products](#products)
    - [Orders](#orders)
    - [Topics and services](#topics-and-services)
    - [Sensors](#sensors)
  - [Scoring](#scoring)


# Wiki | Finals

## Important Dates

- By June 4th, competitors must submit their scripts for evaluation. An extension is possible (no more than a week) as long as the NIST team agrees.
- 7-11 June 2021 - Competitors submission will be evaluated and the results will be posted on June 11th. If an extension is provided, this evaluation week will be pushed back.

## Evaluation

Similar to the qualifiers, competitors' systems will be evaluated headless in docker. More information on the submission and evaluation processes can be found [here](../documentation/automated_evaluation.md).
## Finals Practice

We have provided 3 configuration files for practice. Each configuration file is part of a scenario (see [Scenarios](../documentation/competition_specifications.md#competition-scenarios)). ***Note***: These 3 configuration files consist of the most complex trials and similar ones will used during the evaluation for the finals.

- [finals_practice1.yaml](../../nist_gear/config/trial_config/finals_practice/finals_practice1.yaml):
  - This trial is part or Scenario 1. 
  - Competitors are required to build 2 orders. The first order `order_0` must be built on `agv3` and delivered to `as3`. The second order, `order_1`, is not a high-priority order (priority = 1). `order_1` will be announced once `order_0` is shipped to the correct station. `assembly_pump_blue` must be flipped in `order_1`. Once `order_1` is announced the status of the kitting robot will be set to 0 (check the topic `/ariac/robot_health`), meaning the kitting robot cannot be used to complete `order_1` and the gantry is required for `order_1`. Since the controllers for the kitting robot are still active even though the robot is disabled, competitors should position the kitting robot so it stays out of the way of the gantry during `order_1`.
  - Agility challenges present in this trial are: ***part re-orientation*** and ***faulty products***.
  - Products can be found only in bins and the conveyor belt is not used.
  - The script [finals_practice1.sh](../../nist_gear/script/finals_practice1.sh) can be used to visualize the expected results and scores for this trial.
    - Run it with `rosrun nist_gear final_practice1.sh`
  - The score breakdown for this trial is given below:
  
  
```text
[ INFO] [1622240698.540821194, 18.454000000]: End of trial. Final score: 16
Score breakdown:
[trial_score]
...total trial score: [16]
...total process time: [9.607]
...arms collision?: [0]
...dropped part penalty: [0]
   [order score]
   ...order ID [order_0]
   ...total order score: [8]
   ...completion score: [8]
   ...time taken: [3.091]
   ...priority: [1]
      [kitting score]
      ...shipment type: [order_0_kitting_shipment_0]
      ...completion score: [8]
      ...complete: [true]
      ...submitted: [true]
      ...used correct agv: [true]
      ...sent to correct station: [true]
      ...product type presence score: [2]
      ...product color presence score: [2]
      ...product pose score: [2]
      ...all products bonus: [2]


   [order score]
   ...order ID [order_1]
   ...total order score: [8]
   ...completion score: [8]
   ...time taken: [3.516]
   ...priority: [1]
      [kitting score]
      ...shipment type: [order_1_kitting_shipment_0]
      ...completion score: [8]
      ...complete: [true]
      ...submitted: [true]
      ...used correct agv: [true]
      ...sent to correct station: [true]
      ...product type presence score: [2]
      ...product color presence score: [2]
      ...product pose score: [2]
      ...all products bonus: [2]

  ```

- [finals_practice2.yaml](../../nist_gear/config/trial_config/finals_practice/finals_practice2.yaml):
  - This trial is part of Scenario 2 and consists of 2 orders.
  - `agv1` and `agv2` are loaded with products and are located at station `as2`.
  - `agv4` is loaded with products and is located at station `as4`.
  - `order_0` consists of adding products in the ventilator located at station `as2`. Competitors should use products from `agv1` and `agv2` to do assembly. Each AGV has an extra product just in case the robot drops a product during assembly. If for some reasons the robot drops all products located on the AGVs, competitors can still find those products in `bin1` and `bin2`. Note that `assembly_pump_green` must be flipped for this order.
  - `order_1` is a high-priority order (priority = 3). This order will be announced as soon as one product is placed in the ventilator at `as2` (during `order_0`). As soon as `order_1` is announced, competitors must postpone `order_0` and prioritize `order_1`. Once `order_1` is shipped, competitors should resume `order_0`.
    - To complete `order_1`, competitors should use one of the products located on `agv4`. One extra pump is provided on the AGV in case the robot drops the first pump. Extra pumps can be found in `bin3`.
    - The conveyor belt is not used in this trial.
    - Agility challenges present in this trial are: ***part re-orientation*** and ***in-process order change***.
    - The script [finals_practice2.sh](../../nist_gear/script/finals_practice2.sh) can be used to visualize the expected results and scores for this trial.
      - Run it with `rosrun nist_gear final_practice2.sh`
    - The expected final score for this trial is depicted below.

```text
[ INFO] [1622240698.540821194, 18.454000000]: End of trial. Final score: 56
Score breakdown:
[trial_score]
...total trial score: [56]
...total process time: [12.021]
...arms collision?: [0]
...dropped part penalty: [0]
   [order score]
   ...order ID [order_0]
   ...total order score: [14]
   ...completion score: [14]
   ...time taken: [0]
   ...priority: [1]
      [assembly score]
      ...shipment type: [order_0_assembly_shipment_0]
      ...completion score: [14]
         +2pts x (nb of products with CT & CP): [4]
           +1pt x (nb of products with CC): [2]
             +bonus: 4pts x (nb of products with CT & CP & CC) = [8]
      ...complete: [true]
      ...shipped: [true]
      ...number of products shipped: [2]
      ...has faulty products: [false]
      ...has missing products: [false]
      ...has unwanted products: [false]
      ...number of products shipped with correct type (CT): [2]
      ...number of products shipped with correct pose (CP): [2]
      ...number of products shipped with correct color (CC): [2]
      ...correct assembly station: [true]


   [order score]
   ...order ID [order_1]
   ...total order score: [42]
   ...completion score: [14]
   ...time taken: [0]
   ...priority: [3]
      [assembly score]
      ...shipment type: [order_1_assembly_shipment_0]
      ...completion score: [14]
         +2pts x (nb of products with CT & CP): [4]
           +1pt x (nb of products with CC): [2]
             +bonus: 4pts x (nb of products with CT & CP & CC) = [8]
      ...complete: [true]
      ...shipped: [true]
      ...number of products shipped: [2]
      ...has faulty products: [false]
      ...has missing products: [false]
      ...has unwanted products: [false]
      ...number of products shipped with correct type (CT): [2]
      ...number of products shipped with correct pose (CP): [2]
      ...number of products shipped with correct color (CC): [2]
      ...correct assembly station: [true]

```
  
- [finals_practice3.yaml](../../nist_gear/config/trial_config/finals_practice/finals_practice3.yaml):
  - This trial is part of Scenario 3 and consists of 2 orders.
  - `order_0` is specific to kitting. The order consists of 2 shipments which must be built on `agv1` and `agv2` and delivered to `as2`. The kitting robot will be disabled as soon as 1 product is placed in the kit tray of `agv2`. Competitors are required to use the gantry to finish the kit(s).
    - Products to build the kits are found in bins and on the belt.
  - `order_1` is a not a high-priority order (priority = 1). This order will be announced as soon as soon as `agv2` is shipped to `as2`.
    - To complete `order_1`, competitors should use the products located on `agv1` and `agv2`. 
    - Agility challenges present in this trial are: ***part re-orientation*** and ***faulty products***.
    - The script [finals_practice3.sh](../../nist_gear/script/finals_practice3.sh) can be used to visualize the expected results and scores for this trial.
      - Run it with `rosrun nist_gear final_practice3.sh`
    - The expected final score for this trial is depicted below.

```text
[ INFO] [1622258529.378782466, 37.085000000]: End of trial. Final score: 30
Score breakdown:
[trial_score]
...total trial score: [30]
...total process time: [23.749]
...arms collision?: [0]
...dropped part penalty: [0]
   [order score]
   ...order ID [order_0]
   ...total order score: [16]
   ...completion score: [16]
   ...time taken: [15.389]
   ...priority: [1]
      [kitting score]
      ...shipment type: [order_0_kitting_shipment_0]
      ...completion score: [8]
      ...complete: [true]
      ...submitted: [true]
      ...used correct agv: [true]
      ...sent to correct station: [true]
      ...product type presence score: [2]
      ...product color presence score: [2]
      ...product pose score: [2]
      ...all products bonus: [2]

      [kitting score]
      ...shipment type: [order_0_kitting_shipment_1]
      ...completion score: [8]
      ...complete: [true]
      ...submitted: [true]
      ...used correct agv: [true]
      ...sent to correct station: [true]
      ...product type presence score: [2]
      ...product color presence score: [2]
      ...product pose score: [2]
      ...all products bonus: [2]


   [order score]
   ...order ID [order_1]
   ...total order score: [14]
   ...completion score: [14]
   ...time taken: [0]
   ...priority: [1]
      [assembly score]
      ...shipment type: [order_1_assembly_shipment_0]
      ...completion score: [14]
         +2pts x (nb of products with CT & CP): [4]
           +1pt x (nb of products with CC): [2]
             +bonus: 4pts x (nb of products with CT & CP & CC) = [8]
      ...complete: [true]
      ...shipped: [true]
      ...number of products shipped: [2]
      ...has faulty products: [false]
      ...has missing products: [false]
      ...has unwanted products: [false]
      ...number of products shipped with correct type (CT): [2]
      ...number of products shipped with correct pose (CP): [2]
      ...number of products shipped with correct color (CC): [2]
      ...correct assembly station: [true]
```

## Finals Specifications


### Competition management

- How many trials will be run?
  - The competition will be made up of 15 trial runs.
  - Each trial run will use a unique competition scenario configuration, including any combination of [the released agility challenges](../documentation/agility_challenges.md).

- Will there be a time limit for each trial run?
  - Yes, 500 simulation seconds. With a real-time factor of 1.0, this equates to 500 real seconds. With a real-time factor of 0.5, this equates to 1000 real seconds.
  - Teams can listen to the `/clock` ROS topic to know the simulation time ([see 'Using Simulation Time from the /clock Topic'](http://wiki.ros.org/Clock#Using_Simulation_Time_from_the_.2BAC8-clock_Topic)).
  - The trial will automatically end once the time limit is reached; only shipments that have been submitted by that time will be counted.

- Can each trial run be done with different solutions? E.g., using a scanner over the belt in one trial, but using a logical camera over the belt in another trial?
  - One sensor configuration file is to be used for all trials.
  - Teams must provide a single "system" that will be used for all trials: the same system will be run multiple times in exactly the same way.
  - The system will not have access to the trial number or other information about the trial configuration at startup time.

- Are re-runs ever permitted?
  - The only circumstance in which re-runs will be warranted are if a bug in the simulation impacts the competitor's performance in a trial.
  - If you believe you have been impacted by a bug in the simulation, please open a ticket on Github and we will investigate the issue.

- Is control of the arm permitted before the competition has started?
  - No, control of anything in the environment is not permitted before the `start_competition` service has been called, including the arm and the AGVs.
  - Behavior counter to this will be detected on review of trials.

### Products

- Which storage bins will be used to store products?
  - The same storage (bins and belt) used in the Qualifiers will be used in the Finals.
  In a given trial, some storage bins may be empty and the belt not be used at all.
  - There will be at most one type of product per bin and at most two types of product on the belt.

- Which products will be used in the competition?
  - The 4 models used in Qualifiers will be used in the Finals; no additional products will be added.

- Will storage bins ever contain more than one type of product?
  - No, there will be at most one type of product per bin.

- How can we know the starting configuration of products in the bins?
  - The number of products that are stored in a given bin and their configuration will vary with each trial run.
  - Sensors should be used to detect the product types and their configurations; the information will not be communicated to teams through other means.
  - Within a storage bin, the product type and origination are homogeneous.
  - Product IDs (reported by logical cameras) will be randomized.
  - All products will start in the storage bins un-flipped.

### Orders

- How many orders per trial run can be expected?
  - There will be **at most two orders** per trial run. If two orders are received, the second order can be of priority 1 or 3.

- How do we know the priority of an incoming order?
  - The priority of the order is shown when the order is announced on the topic `ariac/orders`.

- What are the limitations of the orders?
  - Orders may be made up of products available from the bins/belt. Each order may have up to 2 shipments. No shipment will have more than 4 products.

- Will there always be enough products to fulfill the required orders?
  - No. It is not guaranteed that sufficiently many non-faulty products of each type will be available to complete the requested orders. Competitors will have to submit incomplete orders.

### Topics and services

- Which of the services can be used during the Finals?
  - The topics/services listed in [in the ARIAC interface page](../documentation/competition_interface_documentation.md) that are not labelled as "cheats" will be available during the finals.
  - All other interfaces with the simulation will be blocked, and attempts to use them will not be permitted.

### Sensors

- Is there a minimum/maximum number of sensors?
  - A minimum of one sensor is required, no maximum.

- Are there restrictions on the placement of sensors?
  - Sensors can be placed in any free space in the workcell, they do not need to be mounted such that they are touching the conveyor belt/support frame of the storage bin.
  - Sensors must be used in a realistic manner and must not exploit any simulation technicalities such as the logical camera seeing through obstructions.

- Where will the quality control sensors be positioned?
  - The quality control sensors will be positioned above the AGVs in the same location as the qualifiers, for all trials.

## Scoring

- What properties must be satisfied for a product to be scored correctly?
  - Products must be placed on the base of the tray to be scored.
  - Products balancing on top of other products will not be counted.
  - The tolerance of the position/orientation scoring is listed on the [scoring metrics page](../documentation/scoring.md).

- How will the judges' scoring work?
  - In addition to the automated metrics, judges will score teams' systems based on real-world feasibility and agility. The [challenge.gov](https://www.challenge.gov/challenge/ariac/) page details how the judges' scoring will work, and how it will be combined with the automated scoring results.