
.. _TUTORIAL_5:

======================================
Tutorial 5: Moving AGVs
======================================

.. note::
  **Prerequisites**: Tutorial 1 and 4 should be completed before starting this tutorial.

This tutorial covers the following steps:

  - Receive order messages, 
  - Store each order in a list,
  - Find assembly tasks in the list, 
  - Identify AGVs and stations from the assembly tasks
  - Move the AGVs to the stations.

Once this tutorial completed, the package ``competition_tutorials`` should have the following structure:

.. code-block:: bash
    
    competition_tutorials
    ├── CMakeLists.txt                 (updated)
    ├── package.xml
    ├── competition_tutorials
    │   ├── __init__.py
    │   └── competition_interface.py    (new)
    └── src
        ├── start_competition.py        (from tutorial 1)
        ├── read_break_beam_sensor.py   (from tutorial 2)
        ├── read_advanced_camera.py     (from tutorial 3)
        ├── read_orders.py              (from tutorial 4)
        └── move_agvs.py                (new)





Competition Interface
--------------------------------

The competition interface used in this tutorial is shown in :numref:`competitioninterface-tutorial5`.

.. code-block:: python
    :caption: Competition interface for tutorial 5
    :name: competitioninterface-tutorial5

    #!/usr/bin/env python3

    from dataclasses import dataclass
    from typing import List
    import rclpy
    from rclpy.node import Node
    from rclpy.parameter import Parameter

    from ariac_msgs.msg import (
        CompetitionState as CompetitionStateMsg,
        Part as PartMsg,
        Order as OrderMsg,
        AssemblyTask as AssemblyTaskMsg,
    )

    from ariac_msgs.srv import MoveAGV
    from geometry_msgs.msg import PoseStamped, Vector3
    from std_srvs.srv import Trigger


    @dataclass
    class KittingPart:
        '''
        Class to store information about a KittingPartMsg.
        '''
        _quadrant: int
        _part: PartMsg

        @property
        def quadrant(self) -> int:
            '''
            Returns the quadrant of the part.

            Returns:
                int: The quadrant of the part.
            '''
            return self._quadrant

        @property
        def part(self) -> PartMsg:
            '''
            Returns the type of the part.

            Returns:
                PartMsg: The type of the part.
            '''
            return self._part


    @dataclass
    class KittingTask:
        '''
        Class to store information about a KittingTaskMsg.
        '''
        _agv_number: int
        _tray_id: int
        _destination: int
        _parts:  List[KittingPart]

        @property
        def agv_number(self) -> int:
            '''
            Returns the AGV number.

            Returns:
                int: The AGV number.
            '''
            return self._agv_number

        @property
        def tray_id(self) -> int:
            '''
            Returns the tray ID.

            Returns:
                int: The tray ID.
            '''
            return self._tray_id

        @property
        def destination(self) -> int:
            '''
            Returns the destination.

            Returns:
                int: The destination.
            '''
            return self._destination

        @property
        def parts(self) -> List[KittingPart]:
            '''
            Returns the list of parts.

            Returns:
                List[KittingPart]: The list of parts.
            '''
            return self._parts


    @dataclass
    class AssemblyPart:
        '''
        Class to store information about a AssemblyPartMsg.
        '''

        _part: PartMsg
        _assembled_pose: PoseStamped
        _install_direction: Vector3

        @property
        def part(self) -> PartMsg:
            '''
            Returns the type of the part.

            Returns:
                PartMsg: The type of the part.
            '''
            return self._part

        @property
        def assembled_pose(self) -> PoseStamped:
            '''
            Returns the assembled pose of the part.

            Returns:
                PoseStamped: The assembled pose of the part.
            '''
            return self._assembled_pose

        @property
        def install_direction(self) -> Vector3:
            '''
            Returns the install direction of the part.

            Returns:
                Vector3: The install direction of the part.
            '''
            return self._install_direction


    @dataclass
    class AssemblyTask:
        '''
        Class to store information about a AssemblyTaskMsg.
        '''

        _agv_numbers: List[int]
        _station: int
        _parts:  List[AssemblyPart]

        @property
        def agv_numbers(self) -> List[int]:
            '''
            Returns the list of AGV numbers.

            Returns:
                List[int]: The list of AGV numbers.
            '''
            return self._agv_numbers

        @property
        def station(self) -> int:
            '''
            Returns the station.

            Returns:
                int: The station.
            '''
            return self._station

        @property
        def parts(self) -> List[AssemblyPart]:
            '''
            Returns the list of parts.

            Returns:
                List[AssemblyPart]: The list of parts.
            '''
            return self._parts


    @dataclass
    class CombinedTask:
        '''
        Class to store information about a CombinedTaskMsg.
        '''

        _station: int
        _parts:  List[AssemblyPart]

        @property
        def station(self) -> int:
            '''
            Returns the station.

            Returns:
                int: The station.
            '''
            return self._station

        @property
        def parts(self) -> List[AssemblyPart]:
            '''
            Returns the list of parts.

            Returns:
                List[AssemblyPart]: The list of parts.
            '''
            return self._parts


    class Order:
        ''' 
        Class to store one order message from the topic /ariac/orders.
        '''

        def __init__(self, msg: OrderMsg) -> None:
            self.order_id = msg.id
            self.order_type = msg.type
            self.order_priority = msg.priority

            if self.order_type == OrderMsg.KITTING:
                self.order_task = KittingTask(msg.kitting_task.agv_number,
                                            msg.kitting_task.tray_id,
                                            msg.kitting_task.destination,
                                            msg.kitting_task.parts)

            elif self.order_type == OrderMsg.ASSEMBLY:
                self.order_task = AssemblyTask(msg.assembly_task.agv_numbers,
                                            msg.assembly_task.station,
                                            msg.assembly_task.parts)
            elif self.order_type == OrderMsg.COMBINED:
                self.order_task = CombinedTask(msg.combined_task.station, msg.combined_task.parts)
            else:
                self.order_task = None


    class CompetitionInterface(Node):
        '''
        Class for a competition interface node.

        Args:
            Node (rclpy.node.Node): Parent class for ROS nodes

        Raises:
            KeyboardInterrupt: Exception raised when the user uses Ctrl+C to kill a process
        '''
        
        _competition_states = {
            CompetitionStateMsg.IDLE: 'idle',
            CompetitionStateMsg.READY: 'ready',
            CompetitionStateMsg.STARTED: 'started',
            CompetitionStateMsg.ORDER_ANNOUNCEMENTS_DONE: 'order_announcements_done',
            CompetitionStateMsg.ENDED: 'ended',
        }
        '''Dictionary for converting CompetitionState constants to strings'''

        _stations = {
            AssemblyTaskMsg.AS1: "assembly station 1",
            AssemblyTaskMsg.AS2: "assembly station 2",
            AssemblyTaskMsg.AS3: "assembly station 3",
            AssemblyTaskMsg.AS4: "assembly station 4",
        }
        '''Dictionary for converting AssemblyTaskMsg constants to strings'''
        
        def __init__(self):
            super().__init__('competition_interface')

            sim_time = Parameter(
                "use_sim_time",
                rclpy.Parameter.Type.BOOL,
                True
            )

            self.set_parameters([sim_time])

            # Service client for starting the competition
            self._start_competition_client = self.create_client(Trigger, '/ariac/start_competition')

            # Subscriber to the competition state topic
            self._competition_state_sub = self.create_subscription(
                CompetitionStateMsg,
                '/ariac/competition_state',
                self.competition_state_cb,
                10)

            # Store the state of the competition
            self._competition_state: CompetitionStateMsg = None

            # Subscriber to the order topic
            self._orders_sub = self.create_subscription(OrderMsg, '/ariac/orders', self.orders_cb, 10)
            # List of orders
            self._orders = []

        @property
        def orders(self):
            '''Property for the orders list.'''
            return self._orders

        def competition_state_cb(self, msg: CompetitionStateMsg):
            '''Callback for the topic /ariac/competition_state

            Arguments:
                msg -- CompetitionState message
            '''
            # Log if competition state has changed
            if self._competition_state != msg.competition_state:
                self.get_logger().info(
                    f'Competition state is: {CompetitionInterface._competition_states[msg.competition_state]}',
                    throttle_duration_sec=1.0)
            self._competition_state = msg.competition_state

        def start_competition(self):
            '''Function to start the competition.
            '''
            self.get_logger().info('Waiting for competition to be ready')

            if self._competition_state == CompetitionStateMsg.STARTED:
                return
            # Wait for competition to be ready
            while self._competition_state != CompetitionStateMsg.READY:
                try:
                    rclpy.spin_once(self)
                except KeyboardInterrupt:
                    return

            self.get_logger().info('Competition is ready. Starting...')

            # Call ROS service to start competition
            while not self._start_competition_client.wait_for_service(timeout_sec=1.0):
                self.get_logger().info('Waiting for /ariac/start_competition to be available...')

            # Create trigger request and call starter service
            request = Trigger.Request()
            future = self._start_competition_client.call_async(request)

            # Wait until the service call is completed
            rclpy.spin_until_future_complete(self, future)

            if future.result().success:
                self.get_logger().info('Started competition.')
            else:
                self.get_logger().info('Unable to start competition')

        def orders_cb(self, msg: OrderMsg):
            '''Callback for the topic /ariac/orders

            Arguments:
                msg (OrderMsg) -- Order message
            '''
            order = Order(msg)
            self._orders.append(order)
        


        def lock_agv_tray(self, num):
            '''Function to lock the tray of an AGV.

            Arguments:
                num -- AGV number

            Raises:
                KeyboardInterrupt: Exception raised when the user presses Ctrl+C
            '''
            tray_locker = self.create_client(
                Trigger,
                f'/ariac/agv{num}_lock_tray'
            )

            request = Trigger.Request()

            future = tray_locker.call_async(request)

            try:
                rclpy.spin_until_future_complete(self, future)
            except KeyboardInterrupt as kb_error:
                raise KeyboardInterrupt from kb_error

            if future.result().success:
                self.get_logger().info(f'Locked AGV{num}\'s tray')
            else:
                self.get_logger().warn('Unable to lock tray')

        def move_agv_to_station(self, num, station):
            '''Function to move an AGV to a station.

            Arguments:
                num -- AGV number

                station -- Station to move to

            Raises:
                KeyboardInterrupt: Exception raised when the user presses Ctrl+C
            '''
            mover = self.create_client(
                MoveAGV,
                f'/ariac/move_agv{num}')

            request = MoveAGV.Request()

            if station in [AssemblyTaskMsg.AS1, AssemblyTaskMsg.AS3]:
                request.location = MoveAGV.Request.ASSEMBLY_FRONT
            else:
                request.location = MoveAGV.Request.ASSEMBLY_BACK

            future = mover.call_async(request)

            try:
                rclpy.spin_until_future_complete(self, future)
            except KeyboardInterrupt as kb_error:
                raise KeyboardInterrupt from kb_error

            if future.result().success:
                self.get_logger().info(f'Moved AGV{num} to {self._stations[station]}')
            else:
                self.get_logger().warn(future.result().message)






Contents of the competition interface specific to this tutorial are described as follows:

    - ``lock_agv_tray``: Method to lock the tray of an AGV. This method creates a client to the ``/ariac/agv{num}_lock_tray`` service and calls it. The AGV number is passed as an argument to the method.
    - ``move_agv_to_station``: Method to move an AGV to a station. This method creates a client to the ``/ariac/move_agv{num}`` service and calls it. The AGV number and station are passed as arguments to the method.



Create the Executable
--------------------------------

To test this tutorial, create a new file ``move_agvs.py`` in ``competition_tutorials/src``:

.. code-block:: bash

    cd ~/ariac_ws/src/competition_tutorials/src
    touch move_agvs.py
    chmod +x move_agvs.py


Copy the following code in the file ``move_agvs.py``:


.. code-block:: python
    :caption: move_agvs.py
    
    #!/usr/bin/env python3

    import rclpy
    from ariac_msgs.msg import Order as OrderMsg
    from competition_tutorials.competition_interface import CompetitionInterface


    def main(args=None):

        rclpy.init(args=args)
        interface = CompetitionInterface()
        interface.start_competition()

        while not interface.orders:
            try:
                rclpy.spin_once(interface)
            except KeyboardInterrupt:
                break

        for order in interface.orders:
            if order.order_type == OrderMsg.ASSEMBLY:
                for agv in order.order_task.agv_numbers:
                    interface.lock_agv_tray(agv)
                    interface.move_agv_to_station(agv, order.order_task.station)

        interface.destroy_node()
        rclpy.shutdown()

    if __name__ == '__main__':
        main()


The for loop in the ``main`` function iterates through the orders, retrieves orders with assembly tasks, retrieves AGVs for the assembly tasks, locks the tray of the AGVs and moves them to the assembly station. The ``lock_agv_tray`` and ``move_agv_to_station`` methods are defined in the ``CompetitionInterface`` class.

Update CMakelists.txt
^^^^^^^^^^^^^^^^^^^^^^

Update ``CMakeLists.txt`` to add ``read_orders.py`` as an executable.

.. code-block:: cmake

  # Install Python executables
  install(PROGRAMS
    src/start_competition.py
    src/read_break_beam_sensor.py
    src/read_advanced_camera.py
    src/read_orders.py
    src/move_agvs.py
    DESTINATION lib/${PROJECT_NAME}
  )


Run the Executable
--------------------------------

Next, build the package and run the executable.


.. code-block:: bash
    :caption: Terminal 1

    cd ~/ariac_ws
    colcon build
    . install/setup.bash
    ros2 run competition_tutorials move_agvs.py


The node will wait until the competition is ready. In a second terminal, run the following:

.. code-block:: bash
    :caption: Terminal 2

    cd ~/ariac_ws
    . install/setup.bash
    ros2 launch ariac_gazebo ariac.launch.py competitor_pkg:=ariac_tutorials trial_name:=tutorial


Once the environment is loaded and the competition state is ready, the interface node running in Terminal 1 will start the competition and move AGS 1 and 2 to station 1.

Outputs
--------------------------------

Terminal outputs of tutorial 5 are provided below.

.. code-block:: text
    :caption: Terminal outputs
    
    [INFO] [1679043864.680244149] [competition_interface]: Waiting for competition to be ready
    [INFO] [1679043864.681023755] [competition_interface]: Competition state is: ready
    [INFO] [1679043864.681309010] [competition_interface]: Competition is ready. Starting...
    [INFO] [1679043864.683703043] [competition_interface]: Started competition.
    [INFO] [1679043864.692431248] [competition_interface]: Locked AGV1's tray
    [INFO] [1679043871.798302676] [competition_interface]: Moved AGV1 to assembly station 1
    [INFO] [1679043871.799515938] [competition_interface]: Locked AGV2's tray
    [INFO] [1679043878.443151905] [competition_interface]: Moved AGV2 to assembly station 1

