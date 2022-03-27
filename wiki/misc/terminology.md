Wiki | [Home](../../README.md) | [Documentation](../documentation/documentation.md) | [Tutorials](../tutorials/tutorials.md) | [Qualifiers](../qualifiers/qualifier.md) | [Finals](../finals/finals.md)

---

- [Wiki | Terminology](#wiki--terminology)
  - [Assembly](#assembly)
  - [Competitor](#competitor)
  - [Configuration File](#configuration-file)
  - [GEAR](#gear)
  - [Kitting](#kitting)
  - [Order](#order)
  - [Part (or Product)](#part-or-product)
  - [Part Vessel](#part-vessel)
  - [Shipment](#shipment)
  - [Task](#task)
  - [Trial](#trial)
  - [Assembly/Kitting Robots](#assemblykitting-robots)

# Wiki | Terminology

## Assembly

- Assembly is a manufacturing process in which interchangeable parts are added to a product in a sequential manner to create an end product. In ARIAC assembly is simplified by not "forcing" competitors to use a sequence during assembly.

## Competitor

- Persons competing in ARIAC. Sometimes they are referred as "teams" or "participants".

## Configuration File

- There are two types of configuration files used in ARIAC.

  - The `trial configuration files` are YAML files created by the ARIAC organizers. These files are used to set agility challenges as well as some variable parameters for a [trial](##Trial). Each trial has an associated `trial configuration file`. During the development phase competitors are allowed to edit those files. A set of `trial configuration files` are provided to competitors. Competitors are allowed to tweak these files. Understanding the content of the these files is needed if competitors would like to create custom trials.
  - The `user configuration file` is a file provided by competitors in which sensor types and poses are specified. During the development phase, competitors can tweak this file until they are satisfied with their choice of sensors. For the qualifiers and the finals, only one `user configuration file` will be used for all the trials for each competitor.
- Full description of each field found in these files is located in the [YAML Configuration Files](../documentation/configuration_files.md)

## GEAR

- The GEAR (Gazebo Environment for Agile Robots) interface allows for a controlled standardized means of communication between competitors and the simulation environment. To maximize flexibility, GEAR was implemented to be a ROS-based interface. While new features were added to GEAR for each iteration of ARIAC, its structure has remained consistent across competitions. With GEAR, competitors  implement their system in a variety of supported programming languages. Additionally, this approach was chosen to isolate the use of a simulated environment as an implementation detail. Competitors' systems never communicate directly with the Gazebo simulator, but instead, with GEAR which in turn communicated with the simulator via a Gazebo-ROS integration layer.

## Kitting

- Kitting is the process which groups separate  but related items (parts in ARIAC) as one unit. Kits are built in kit trays located on AGVs.

## Order

- An order is an instruction with the type, the color, and the pose of each part to be placed in a kit (for kitting) or in a briefcase (for assembly). The order also specifies where to build kits, which is usually either on the AGVs (for kitting) or at assembly stations (for assembly).  As one can note, an order is typically a goal state that the robot needs to  reach through pick-and-place operations. An order consists of at least one shipment. When an order consists of multiple shipments, competitors's systems must build and ship the same shipment multiple times.

## Part (or Product)

- A part (also called product) is an object used during kitting and assembly. In ARIAC 2022, the same parts are used in both kitting and assembly. Parts from ARIAC 2021 are reused this year. The dimension of some part types have been altered.

## Part Vessel

- A part vessel is a physical container in the environment where parts can spawn. In ARIAC 2022, part vessels are bins, the conveyor belt, or AGV trays.

## Shipment

- A shipment is an instance of an order. An order has at least one shipment and up to two shipments per task. Completing all the shipments within an order leads to the completion of the order itself.
  - ARIAC 2021 has two types of shipments: `kitting shipments` for the kitting task and `assembly shipments` for the assembly task.

## Task

- The scope of the manufacturing mission. ARIAC 2022 consists of two tasks: kitting and assembly.

## Trial

- A trial is a single run of the simulation in which at least one order is  described. A trial may have up to two orders where the second order is either a regular order of an order of highest priority. In ARIAC, trials come in the form of YAML-formatted configuration files. Examples of trial configuration files can be found in the folder `ARIAC/nist_gear/config/trial_config`.
  - Although these files are provided by the ARIAC organizers, competitors should understand the content of these files. Doing so will help competitors what to expect in a trial. Competitors can also modify provided trial configuration files during development.
  - During qualifiers and finals, competitors' systems will be evaluated with unseen trial configuration files. Competitors will not have the ability to modify these files.

## Assembly/Kitting Robots

- Whenever kitting robot is mentioned in the wiki, it is referencing the UR10 arm mounted on the linear rail. This robot can only perform kitting, hence the name.
- The assembly robot (or gantry robot) is the robot mounted on rails attached to the ceiling. Even though technically this robot's main purpose is to do assembly, it can also perform kitting and in some situations, it will be the only robot allowed to do both kitting and assembly.

---

Wiki | [Home](../../README.md) | [Documentation](../documentation/documentation.md) | [Tutorials](../tutorials/tutorials.md) | [Qualifiers](../qualifiers/qualifier.md) | [Finals](../finals/finals.md)
