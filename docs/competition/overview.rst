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




2. **terminal 2**: This command starts the competitor's control system (CCS), which is the software implemented by the competitor to compete in ARIAC. This command can be a ``ros2 run`` or ``ros2 launch`` command. The first task of the CCS is to start the competition with the following service call:

    .. code-block:: bash

        ros2 service call /ariac/start_competition std_srvs/srv/Trigger

    The state of the competition must be in the state ``READY`` before this service can be called. The call to this service starts the robot controllers, activates all sensors, start the conveyor belt (if part of the trial), and starts the global challenges (if part of the trial). Orders will be announced on the topic ``/ariac/orders``. The result of the call will set the state of the competition to ``STARTED``.

    Once orders are announced, the CCS fulfills the orders and submits them. Order announcements can be time based, part placement based, or order submission based. More information on these conditions can be found in :ref:`_CONDITIONS`. Agility challenges can also be announced with these conditions. More information on agility challenges can be found in :ref:`_AGILITYCHALLENGES`.

3. **announce order(s)**: The AM will announce orders on the topic ``/ariac/orders``. The CCS will  need to subscribe to the topic to receive the orders. If all orders have been announced, the AM will set the state of the competition to ``ORDER_ANNOUNCEMENTS_DONE``. This state does not mean that the competition is over. The CCS may still be working on orders that were announced earlier.

4. **work on order(s)**: During this phase, the CCS will perform different activities in order to fulfill the orders. The AM may announce new orders or start agility challenges based on the state of the workcell.

5. **submit order(s)**: After orders are completed they are submitted by the CCS. Order submission may announce new orders and/or start agility challenges.  

    .. warning:: 
        
        To submit a kitting order, the CCS needs to ensure the AGV is at the warehouse before calling the service to submit an order.
        The AGV status can be retrieved by subscribing to the topic ``/ariac/agv{n}_status`` (see :ref:`COMMUNICATIONS` for more information).

6. **end competition**: Once the CCS have submitted all orders, they need to call the following service to end the competition.

    .. code-block:: bash

        ros2 service call /ariac/end_competition std_srvs/srv/Trigger


    The result of the call will set the state of the competition to ``ENDED``.

7. **calculate scoring**: The last phase of a trial is the computation of the score for the trial. The score is computed using the formulas described in the :ref:`SCORING` section. The score is thendisplayed in the terminal.
