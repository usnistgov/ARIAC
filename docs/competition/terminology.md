# Terminology

## Competitors

Persons competing in ARIAC. Sometimes competitors are referred as "teams" or "participants".

## Trial

Each run of the competition is called a trial. The configuration for that trial is defined by a
configuration file (YAML). Competitors do not and must not directly read trial files but needs to use topics and services provided by the competition environment. More information on how to use the competition environment can be found in the [ROS Communication Overview](ros_communication.md).

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

Assembly is a manufacturing process in which interchangeable parts are added to a product in a sequential manner to create an end product. In ARIAC, assembly is simplified by not "forcing" competitors to use a sequence during assembly. Competitors can place parts in an insert in any order. For an assembly task, competitors are expected to use parts located on an AGV and assemble those parts at one of the four assembly stations.

## Combined Task

A combined task consists of a Kitting Task and an Assembly Task. In a combined task, only the Assembly Task is scored. Competitors have to find a way to move parts from their original locations to the assembly station.

## Insert
