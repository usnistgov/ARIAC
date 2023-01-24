# Terminology

## Competitors

Persons competing in ARIAC. Sometimes competitors are referred as "teams" or "participants".

## Trial

Each run of the competition is called a trial. The configuration for that trial is defined by a
configuration file (YAML). Competitors do not and must not directly read trial files. An example of a trial file is
provided in `sample.yaml`.

A trial consists of a time limit (-1 for no time limit), information on kitting tray locations,
part locations, orders, and challenges.

## Part

Parts are used during pick-and-place operations. There are four available parts (battery, pump, regulator, and sensor) and each part can be one of five possible colors (red, green, blue, orange, and purple).

At the start of the environment, parts can be located in 3 different locations:

1. Bins.
2. AGVs.
3. Conveyor belt.

## Order

An order is an intruction containing information on a task ([kitting task](#kitting-task), [assembly task](#assembly-task), or [combined task](#combined-task)). Each task consists of at least one part of a specific color and type. 

## Kitting Task

Kitting is the process which groups separate but related [parts](#part) as one unit. For a kitting task, competitors are expected to: 

1. Place a kit tray onto one of the four AGVs.
2. Place parts onto that kit tray in a specific quadrant.
3. Direct the AGV to the warehouse.
4. Evaluate the submitted kit for scoring.

## Assembly Task

Assembly is a manufacturing process in which interchangeable parts are added to a product in a sequential manner to create an end product. 

In ARIAC, assembly is simplified by not "forcing" competitors to use a sequence during assembly. Competitors can place parts in an insert in any order.

For an assembly task, competitors are expected to use parts located on an AGV and assemble those parts at one of the four assembly stations.

## Combined Task

A combined task consists of a Kitting Task and an Assembly Task. In a combined task, only the Assembly Task is scored. Competitors have to find a way to move parts from their original locations to the assembly station.

<!-- ### Announcement
An order has unique ID and can be announced from one of the following conditions:
- **Time**: Time since the start of the competition
- **On part placement**: When a part is placed in a kitting tray or in an insert.
- **On submission**: When another order has been submitted.

### Priority
An order has a priority. When set to `false` the order is a regular order and when
set to `true`, the order is of high priority.

### Examples
An example of an order consisting of a kitting task is provided below.

```yaml
orders:
  - id: 'MMB30H56'
    type: 'kitting'
    announcement:
      time_condition: 0
    priority: false
    challenges:
      flipped_part:
        quadrant: 1
    kitting_task:
      agv_number: 2
      tray_id: 3
      destination: 'warehouse'
      products:
        - type: 'battery'
          color: 'blue'
          quadrant: 1
        - type: 'pump'
          color: 'red'
          quadrant: 3
```

An example of an order consisting of an assembly task is provided below.

```yaml
orders:
  - id: '1X7K29EV'
    type: 'assembly'
    announcement:
      part_place_condition:
        color: 'green'
        type: 'sensor'
        agv: 4
    priority: true
    assembly_task:
      agv_number: [4]
      station: 'as4'
      products:
        - type: 'sensor'
          color: 'green'
          assembled_pose: # relative to breifcase frame
            xyz: [0.405, 0.164, 0.110]
            rpy: ['pi/2', 0, 0]
          assembly_direction: [-1, 0, 0] # unit vector in briefcase frame
```

An example of an order consisting of a combined task is provided below.
```yaml
- id: '2Y7K29EV'
    type: 'combined'
    announcement:
      submission_condition:
        order_id: '1X7K29EV'
    priority: false
    combined_task:
      station: 'as2'
      products:
        - type: 'sensor'
          color: 'blue'
          assembled_pose: # relative to briefcase frame
            xyz: [0.405, 0.164, 0.110]
            rpy: ['pi/2', 0, 0]
          assembly_direction: [-1, 0, 0] # unit vector in briefcase frame
``` -->