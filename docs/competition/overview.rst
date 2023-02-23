Overview of the Competition
===========================

.. figure:: ../images/ARIAC2023Flowchart.jpg
   :scale: 50 %
   :alt: flowchart
   :align: center
   :figclass: align-center
   :name: flowchart
   :class: no-border
   

   Flowchart showing the interactions between the :term:`CCS<Competitor Control System (CCS)>` and the :term:`AM<ARIAC Manager (AM)>`.

   


:numref:`flowchart` provides an overview of the competition. 
The competition consists of two main actors, the :term:`CCS<Competitor Control System (CCS)>` and the :term:`AM<ARIAC Manager (AM)>`. 
The competition is set to different states while it is running and the CCS needs to subscribe to the topic ``/ariac/competition_state`` to properly implement the programming logic. 

Start Commands
--------------

To compete in ARIAC, competitors have to issue two commands in two different terminals.

1. **terminal 1**: In the first terminal, competitors **start the trial** with the following command:

    .. code-block:: bash

        ros2 launch ariac_gazebo ariac.launch.py trial_config:=<trial_yaml_file> sensor_config:=<sensor_yaml_file>

    ``trial_yaml_file`` and ``sensor_yaml_file`` are yaml file names without the ``.yaml`` suffix. If ``trial_yaml_file`` is not provided, it defaults to ``kitting.yaml``. If ``sensor_yaml_file`` is not provided, it defaults to ``sensors.yaml``
    
    This command starts the Gazebo simulation environment and the ARIAC manager (AM), the latter handles the communications between the competitor's control system (CCS) and the ARIAC software. The state of the competition changes multiple times during a trial. The state of the competition is published to the topic ``ariac/competition_state`` which uses  ``CompetitionState.msg`` (see :ref:`COMPETITIONSTATEMSG`). The CCS needs to subscribe to this topic to properly implement the programming logic.


    .. code-block:: bash
        :caption: CompetitionState.msg
        :name: CompetitionStateMsg

        uint8 IDLE=0    # competition cannot be started yet by the competitor
        uint8 READY=1   # competition can be started by the competitor
        uint8 STARTED=2 # competition has been started
        uint8 ORDER_ANNOUNCEMENTS_DONE=3 # all order announcements have been announced
        uint8 ENDED=4   # competition has ended

        uint8 competition_state # IDLE, READY, STARTED, ORDER_ANNOUNCEMENTS_DONE, ENDED




2. **terminal 2**: Once the trial is started, competitors start their control system (CCS), which will handle orders and challenges during the competition. Starting the CCS can be a ``ros2 run`` or ``ros2 launch`` command. The first task of the CCS is to start the competition with the service ``/ariac/start_competition``. This service is hosted by the AM and is of type ``std_srvs/srv/Trigger``. 

    The state of the competition must be ``READY`` before this service can be called. The call to this service starts the robot controllers, activates all sensors, starts the conveyor belt (if used in the trial), and starts the global challenges (if used in the trial). Orders will be announced on the topic ``/ariac/orders``. The result of the call will set the state of the competition to ``STARTED``.

    Once orders are announced, the CCS fulfills and submits orders. Order announcements can be time based, part placement based, or order submission based. More information on these conditions can be found in :ref:`CONDITIONS`. Agility challenges can also be announced with these conditions. More information on agility challenges can be found in :ref:`AGILITY_CHALLENGES`. To submit orders, the CCS needs to use the service ``/ariac/submit_order`` which uses the ID of the order as an argument (see :ref:`SubmitOrderSrv`).

    .. code-block:: bash
        :caption: SubmitOrder.srv
        :name: SubmitOrderSrv

        string order_id
        ---
        bool success
        string message

    .. warning:: 
        
        To submit a kitting order, the CCS first has to move the AGV to the warehouse with the service ``/ariac/move_agv{n}`` (see :ref:`MoveAGVSrv`). Once the AGV is at the warehouse, then the submission service should be called. To know the location of an AGV in the workcell, the CCS has to subscribe to the topic ``/ariac/agv{n}_status``, which uses ``AGVStatus.msg`` (see :ref:`AGVSTATUSMSG`).

    .. code-block:: bash
        :caption: MoveAGV.srv
        :name: MoveAGVSrv

        int8 KITTING=0
        int8 ASSEMBLY_FRONT=1
        int8 ASSEMBLY_BACK=2 
        int8 WAREHOUSE=3 

        int8 location # KITTING, ASSEMBLY_FRONT, ASSEMBLY_BACK, WAREHOUSE
        ---
        bool success
        string message

    .. code-block:: bash
        :caption: AGVStatus.msg
        :name: AGVStatusMsg

        uint8 KITTING=0
        uint8 ASSEMBLY_FRONT=1
        uint8 ASSEMBLY_BACK=2
        uint8 WAREHOUSE=3
        uint8 UNKNOWN=99

        int8 location # KITTING, ASSEMBLY_FRONT, ASSEMBLY_BACK, WAREHOUSE, UNKNOWN
        float64 position
        float64 velocity

    Once all orders have been submitted, the CCS calls the service ``/ariac/end_competition``. This service is hosted by the AM and is of type ``std_srvs/srv/Trigger``. The result of the call will set the state of the competition to ``ENDED``. The CCS can then exit. The AM will then compute the scoring for the current trial (see :ref:`SCORING` section), end the trial, and save the results. Before calling the service to end the competition, the CCS needs to ensure that all orders have been announced. The state of competition is set to ``ORDER_ANNOUNCEMENTS_DONE`` when all orders from the trial have been announced. 

