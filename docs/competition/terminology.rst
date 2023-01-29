#############
Terminology
#############

.. glossary::
    :sorted:

    Agility Challenge
      The ARIAC Agility Challenge is a competition that tests the ability of a robot to perform a series of tasks in a dynamic environment. The tasks are designed to test the robot's ability to perform pick-and-place operations, assembly, and kitting. The competition is designed to be a testbed for the development of algorithms that can be used in a real-world manufacturing environment.

    Competitor Control System (CCS)
      The competitor control system (CCS) is the software that is provided by competitors. The CCS is responsible for communicating with the competition environment and executing the tasks. 
    
    ARIAC Manager (AM)
      The ARIAC manager is the interface provided to competitors. The AM includes the simulation environment and ROS interfaces.
    
    Competitor
      Person competing in ARIAC. Sometimes competitor is referred as "team" or "participant".

    Trial
      Each run of the competition is called a trial. The configuration for that trial is defined by a configuration file (YAML). Competitors do not and must not directly read trial files but needs to use topics and services provided by the competition environment. More information on how to use the competition environment can be found in the :ref:`Communication Overview`.

    Part
      Parts are used during pick-and-place operations. There are four available parts (battery, pump, regulator, and sensor) and each part can be one of five possible colors (red, green, blue, orange, and purple).

    Order
      An order is an intruction containing information on a task (`target to kitting task_`, `target to assembly task_`, or `target to combined task_`). Each task consists of at least one part of a specific color and type.
    
    Kitting Task
      Kitting is the process which groups separate but related [parts](#part) as one unit. For a kitting task, competitors are expected to: 
      
      1. Place a kit tray onto one of the four AGVs.
      2. Place parts onto that kit tray in a specific quadrant.
      3. Direct the AGV to the warehouse.
      4. Evaluate the submitted kit for scoring.

    Assembly Task
      Assembly is a manufacturing process in which interchangeable parts are added to a product in a sequential manner to create an end product. In ARIAC, assembly is simplified by not "forcing" competitors to use a sequence during assembly. Competitors can place parts in an insert in any order. For an assembly task, competitors are expected to use parts located on an AGV and assemble those parts at one of the four assembly stations.

    Combined Task
      A combined task consists of a Kitting Task and an Assembly Task. In a combined task, only the Assembly Task is scored. Competitors have to find a way to move parts from their original locations to the assembly station.
      
    Insert
      An insert is a container that holds parts.