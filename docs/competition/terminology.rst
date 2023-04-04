#############
Terminology
#############

.. glossary::
    :sorted:

    Agility Challenge
      The ARIAC Agility Challenge is a competition that tests the ability of a robot to perform a series of tasks in a dynamic environment. The tasks are designed to test the robot's ability to perform pick-and-place operations, assembly, and kitting. The competition is designed to be a testbed for the development of algorithms that can be used in a real-world manufacturing environment.

    Competitor Control System (CCS)
      The competitor control system (CCS) is the software that is provided by :term:`competitors<Competitor>`. The CCS is responsible for communicating with the competition environment and executing the tasks. 

    ARIAC Manager (AM)
      The ARIAC manager (AM) is the interface provided to :term:`competitors<Competitor>`. The AM includes the simulation environment and ROS interfaces.

    Competitor
      Person competing in ARIAC. Sometimes competitor is referred as "team" or "participant".

    Automated Guided Vehicle (AGV)
      An automated guided vehicle (AGV) is a mobile robot that follows markers or wires in the floor, or uses vision, magnets, or lasers for navigation. AGVs are used to transport :term:`parts<Part>` from one location to another.



    Trial
      Each run of the competition is called a trial. The configuration for that trial is defined by a configuration file (YAML). :term:`Competitors<Competitor>` do not and must not directly read trial files but needs to use topics and services provided by the competition environment. More information on how to use the competition environment can be found in the 
      :doc:`ros_communication`.

    Part
      Parts are used during pick-and-place operations. There are four available parts (battery, pump, regulator, and sensor) and each part can be one of five possible colors (red, green, blue, orange, and purple).

    Order
      An order is an instruction containing information on a task ( :term:`Kitting Task`, 
      :term:`Assembly Task`, or 
      :term:`Combined Task`,). Each task consists of at least one 
      :term:`part<Part>` of a specific color and type.

    Kitting Task
      Kitting is the process which groups separate but related :term:`parts<Part>` as one unit. For a kitting task, :term:`competitors<Competitor>` are expected to: 
      
        #. Place a kit tray onto one of the four AGVs.
        #. Place parts onto that kit tray in a specific quadrant.
        #. Direct the AGV to the warehouse.
        #. Evaluate the submitted kit for scoring.

    Assembly Task
      Assembly is a manufacturing process in which interchangeable :term:`parts<Part>` are added to a product in a sequential manner to create an end product. In ARIAC, assembly is simplified by not "forcing" :term:`competitors<Competitor>` to use a sequence during assembly. :term:`Competitors<Competitor>` can place :term:`parts<Part>` in an insert in any order. For an assembly task, :term:`competitors<Competitor>` are expected to use :term:`parts<Part>` located on an AGV and assemble those parts at one of the four assembly stations.

    Combined Task
      A combined task consists of a :term:`Kitting Task` and an 
      :term:`Assembly Task`. In a combined task, only the Assembly Task is scored. Competitors have to find a way to move :term:`parts<Part>` from their original locations to the assembly station.
      
    Insert
      An insert is a container that holds :term:`parts<Part>`.
