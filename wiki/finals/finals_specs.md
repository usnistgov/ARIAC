-------------------------------------------------
- Wiki | [Home](../../README.md) | [Documentation](../documentation/documentation.md) | [Tutorials](../tutorials/tutorials.md) | [Qualifiers](../qualifiers/qualifier.md) | [Finals](../finals/finals.md)
-------------------------------------------------

# Wiki|Finals|Finals Specifications #

<!-- [See this page for details of how the Finals will be run.](https://bitbucket.org/osrf/ariac/wiki/2019/finals) -->

## *Competition management*

### **How many trials will be run?**

- The competition will be made up of 15 trial runs.
- Each trial run will use a unique competition scenario configuration, including any combination of [the released agility challenges](https://bitbucket.org/osrf/ariac/wiki/2019/agility_challenges).

### **Will there be a time limit for each trial run?**

- Yes, 500 simulation seconds. With a real-time factor of 1.0, this equates to 500 real seconds. With a real-time factor of 0.5, this equates to 1000 real seconds. 
- Teams can listen to the `/clock` ROS topic to know the simulation time ([see 'Using Simulation Time from the /clock Topic'](http://wiki.ros.org/Clock#Using_Simulation_Time_from_the_.2BAC8-clock_Topic)).
- The trial will automatically end once the time limit is reached; only shipments that have been _submitted_ by that time will be counted.


### **Can each trial run be done with different solutions? E.g., using a scanner over the belt in one trial, but using a logical camera over the belt in another trial?**

- One sensor configuration file is to be used for all trials. 
- Teams must provide a single "system" that will be used for all trials: the same system will be run multiple times in exactly the same way. 
- The system will not have access to the trial number or other information about the trial configuration at startup time.

### **Are re-runs ever permitted?**

- The only circumstance in which re-runs will be warranted are if a bug in the simulation impacts the competitor's performance in a trial.
- If you believe you have been impacted by a bug in the simulation, please [open an issue here](https://bitbucket.org/osrf/ariac/issues?status=new&status=open) to the competition controllers are aware of it.

### **Is control of the arm permitted before the competition has started?**

- No, control of anything in the environment is not permitted before the `start_competition` service has been called, including the arm and the AGVs.
- Behavior counter to this will be detected on review of trials.

## *Products*

### **Which storage bins will be used to store products?**

- The same storage bins used in the Qualifiers will be used in the Finals.
  In a given trial, some storage bins may be empty.
- There will be at most one type of product per bin.

### **Which products will be used in the competition?**
The five models used in Qualifiers will be used in the Finals; no additional products will be added.

### **Will storage bins ever contain more than one type of product?**
No, there will be at most one type of product per bin.

### **How can we know the starting configuration of products in the bins?**
- The number of products that are stored in a given bin and their configuration will vary with each trial run.
- Sensors should be used to detect the product types and their configurations; the information will not be communicated to teams through other means.
- Within a storage bin, the product type and origination are homogeneous.
- Product IDs (reported by logical cameras) will be randomized.
- All products will start in the storage bins un-flipped.

## *Orders*

### **How many orders per trial run can be expected?**

There will be at most two orders per trial run. If two orders are received, the second of those is the high priority order.

### **How do we know the priority of an incoming order?**

The most recently received order is the highest priority.

### **What are the limitations of the orders?**

Orders may be made up of products available from the bins. Each order may have up to 2 shipments. No shipment will have more than 8 products. The only product that will be requested to be flipped is the pulley product.

### **Will there always be enough products to fulfill the required orders?**

No.
It is not guaranteed that sufficiently many non-faulty products of each type will be available to complete the requested orders.

## *Topics and services*

### **Which of the services can be used during the Finals? **
- The topics/services listed in [in the ARIAC interface page](https://bitbucket.org/osrf/ariac/wiki/2019/competition_interface_documentation) that are not labelled as "cheats" will be available during the finals. 
- All other interfaces with the simulation will be blocked, and attempts to use them will not be permitted.

## *Sensors*

### **Is there a minimum/maximum number of sensors?**

A minimum of one sensor is required, no maximum.

### **Are there restrictions on the placement of sensors?**

- Sensors can be placed in any free space in the workcell, they do not need to be mounted such that they are touching the conveyor belt/support frame of the storage bin.
- Sensors must be used in a realistic manner and must not exploit any simulation technicalities such as the logical camera seeing through obstructions.

### **Where will the quality control sensors be positioned?**

The quality control sensors will be positioned above the AGVs in the same location as the qualifiers, for all trials.

## *Scoring*

### **What properties must be satisfied for a product to be scored correctly?**

- Products must be placed on the base of the tray to be scored.
- Products balancing on top of other products will not be counted.
- The tolerance of the position/orientation scoring is listed on the [scoring metrics page](https://bitbucket.org/osrf/ariac/wiki/2019/scoring).

### **How will the judges' scoring work?**

In addition to the automated metrics, judges will score teams' systems based on real-world feasibility and agility.
The [challenge.gov](https://www.challenge.gov/challenge/ariac/) page details how the judges' scoring will work, and how it will be combined with the automated scoring results.

-------------------------------------------------
- Wiki | [Home](../../README.md) | [Documentation](../documentation/documentation.md) | [Tutorials](../tutorials/tutorials.md) | [Qualifiers](../qualifiers/qualifier.md) | [Finals](../finals/finals.md)

-------------------------------------------------
