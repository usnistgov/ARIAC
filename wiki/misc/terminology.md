Wiki | [Home](../../README.md) | [Documentation](../documentation/documentation.md) | [Tutorials](../tutorials/tutorials.md) | [Qualifiers](../qualifiers/qualifier.md) | [Finals](../finals/finals.md)

-------------------------------------------------

- [Wiki | Terminology](#wiki--terminology)
  - [Assembly](#assembly)
  - [GEAR](#gear)
  - [Kitting](#kitting)
  - [Order](#order)
  - [Shipment](#shipment)
  - [Trial](#trial)
  - [Kitting and Assembly Robot](#kitting-and-assembly-robot)

# Wiki | Terminology

## Assembly

- Assembly is a manufacturing process in which interchangeable parts are added to a product in a sequential manner to create an end product. In ARIAC assembly is simplified by not "forcing" competitors to use a sequence during assembly.

## GEAR

- The GEAR (Gazebo Environment for Agile Robots) interface allows for a controlled standardized means of communication between competitors and the simulation environment. To maximize flexibility, GEAR was implemented to be a ROS-based interface. While new features were added to GEAR for each iteration of ARIAC, its structure has remained consistent across competitions. With GEAR, competitors  implement their system in a variety of supported programming languages. Additionally, this approach was chosen to isolate the use of a simulated environment as an implementation detail. Competitors' systems never communicate directly with the Gazebo simulator, but instead, with GEAR which in turn communicated with the simulator via a Gazebo-ROS integration layer.

## Kitting

- Kitting is the process which groups separate  but related items (parts in ARIAC) as one unit. Kits are built in kit trays located on AGVs.

## Order

- An order is an instruction with the type, the color, and the pose of each part to be placed in a kit (for kitting) or in a briefcase (for assembly). The order also specifies where to build kits, which is usually either on the AGVs (for kitting) or at assembly stations (for assembly).  As one can note, an order is typically a goal state that the robot needs to  reach through pick-and-place operations. An order consists of at least one shipment. When an order consists of multiple shipments, competitors's systems must build and ship the same shipment multiple times.

## Shipment

- A shipment is an instance of an order. An order has at least one shipment and up to two shipments. Completing all the shipments within an order leads to the completion of the order itself.
  - ARIAC 2021 has two types of shipments: `kitting shipments` and `assembly shipments`.

## Trial

- A trial is a single run of the simulation in which at least one order is  described. A trial may have up to two orders where the second order is usually of highest priority. In ARIAC, trials come in the form of YAML-formatted configuration files. Examples of trial configuration files can be found in `ARIAC/nist_gear/config/trial_config`.
  - Although these files are provided by the ARIAC organizers, competitors should understand the content of these files. Doing so will help competitors what to expect in a trial. Competitors can also modify provided trial configuration files during development.
  - During qualifiers and finals, competitors' systems will be evaluated with unseen trial configuration files. Competitors will not have the ability to modify these files.

## Kitting and Assembly Robot

- Whenever kitting robot is mentioned in the wiki, it is referencing the UR10 arm mounted on the linear rail. This robot can only perform kitting, hence the name.
- The assembly robot is the gantry robot. Even though technically this robot's main purpose is to do assembly, it can also perform kitting. We coul have picked a different name for this robot (maybe gantry?) but it will not sound as cool as "assembly robot".

<!---You can use the [GEAR/ARIAC support forum](https://discourse.ros.org/c/ariac-users) for public discussions about the competition in which other competition participants may participate.-->

-------------------------------------------------
Wiki | [Home](../../README.md) | [Documentation](../documentation/documentation.md) | [Tutorials](../tutorials/tutorials.md) | [Qualifiers](../qualifiers/qualifier.md) | [Finals](../finals/finals.md)
