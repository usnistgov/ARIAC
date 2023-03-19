.. _TUTORIAL_5:

.. only:: builder_html or readthedocs

.. role:: inline-python(code)
    :language: python

.. role:: inline-file(file)

.. role:: inline-tutorial(file)

.. role:: inline-bash(code)
    :language: bash

=========================================================
Tutorial 5: Move AGVs to Stations
=========================================================

.. admonition:: Source Code for Tutorial 5
  :class: attention
  :name: tutorial_5
  
  `https://github.com/jaybrecht/ariac_tutorials/tree/tutorial_5 <https://github.com/jaybrecht/ariac_tutorials/tree/tutorial_5>`_ 

  .. code-block:: bash
    
        cd ~/ariac_ws/ariac_tutorials
        git checkout tutorial_5


This tutorial covers the following steps:

  - Parse received orders to identify assembly tasks, 
  - identify the AGVs that need to be moved to the assembly station,
  - Lock the tray and parts so that they do not fall off the AGV,
  - Move the AGV to the assembly station.

The final state of the package :inline-file:`ariac_tutorials` for :inline-tutorial:`tutorial 5` is as follows:

.. code-block:: text
    :emphasize-lines: 15
    :class: no-copybutton
    
    ariac_tutorials
    ├── CMakeLists.txt
    ├── package.xml
    ├── config
    │   └── sensors.yaml
    ├── ariac_tutorials
    │   ├── __init__.py
    │   ├── utils.py
    │   └── competition_interface.py
    └── nodes
        ├── tutorial_1.py
        ├── tutorial_2.py
        ├── tutorial_3.py
        ├── tutorial_4.py
        └── tutorial_5.py






Competition Interface
--------------------------------

The competition interface used in this tutorial is shown in :numref:`competitioninterface-tutorial5`.

.. code-block:: python
    :caption: competition_interface.py
    :name: competitioninterface-tutorial5
    :linenos:

    #!/usr/bin/env python3

    import rclpy
    from rclpy.node import Node
    from rclpy.parameter import Parameter

    from ariac_msgs.msg import (
        CompetitionState as CompetitionStateMsg,
        Part as PartMsg,
        Order as OrderMsg,
        AssemblyPart as AssemblyPartMsg,
        AssemblyTask as AssemblyTaskMsg,
        AGVStatus as AGVStatusMsg,
    )

    from std_srvs.srv import Trigger
    from ariac_msgs.srv import MoveAGV

    from ariac_tutorials.utils import (
        KittingTask,
        Order,
        KittingPart,
        AssemblyTask,
        CombinedTask
    )


    class CompetitionInterface(Node):
        '''
        Class for a competition interface node.

        Args:
            Node (rclpy.node.Node): Parent class for ROS nodes

        Raises:
            KeyboardInterrupt: Exception raised when the user uses Ctrl+C to kill a process
        '''

        _part_colors = {
            PartMsg.RED: 'red',
            PartMsg.BLUE: 'blue',
            PartMsg.GREEN: 'green',
            PartMsg.ORANGE: 'orange',
            PartMsg.PURPLE: 'purple',
        }

        _part_colors_emoji = {
            PartMsg.RED: '🟥',
            PartMsg.BLUE: '🟦',
            PartMsg.GREEN: '🟩',
            PartMsg.ORANGE: '🟧',
            PartMsg.PURPLE: '🟪',
        }

        '''Dictionary for converting PartColor constants to strings'''

        _part_types = {
            PartMsg.BATTERY: 'battery',
            PartMsg.PUMP: 'pump',
            PartMsg.REGULATOR: 'regulator',
            PartMsg.SENSOR: 'sensor',
        }
        '''Dictionary for converting PartType constants to strings'''

        _competition_states = {
            CompetitionStateMsg.IDLE: 'idle',
            CompetitionStateMsg.READY: 'ready',
            CompetitionStateMsg.STARTED: 'started',
            CompetitionStateMsg.ORDER_ANNOUNCEMENTS_DONE: 'order_announcements_done',
            CompetitionStateMsg.ENDED: 'ended',
        }
        '''Dictionary for converting CompetitionState constants to strings'''

        _destinations = {
            AGVStatusMsg.KITTING: 'kitting station',
            AGVStatusMsg.ASSEMBLY_FRONT: 'front assembly station',
            AGVStatusMsg.ASSEMBLY_BACK: 'back assembly station',
            AGVStatusMsg.WAREHOUSE: 'warehouse',
        }
        '''Dictionary for converting AGVDestination constants to strings'''

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
                self._competition_state_cb,
                10)

            # Store the state of the competition
            self._competition_state: CompetitionStateMsg = None

            # Subscriber to the order topic
            self._orders_sub = self.create_subscription(OrderMsg, '/ariac/orders', self._orders_cb, 10)
            # List of orders
            self._orders = []
            # Flag for parsing incoming orders
            self._parse_incoming_order = False

        @property
        def parse_incoming_order(self):
            '''Property for the parse_incoming_order flag.'''
            return self._parse_incoming_order

        @parse_incoming_order.setter
        def parse_incoming_order(self, value: bool):
            self._parse_incoming_order = value

        def _competition_state_cb(self, msg: CompetitionStateMsg):
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

        def _orders_cb(self, msg: OrderMsg):
            '''Callback for the topic /ariac/orders

            Arguments:
                msg (OrderMsg) -- Order message
            '''
            order = Order(msg)
            self._orders.append(order)
            if self._parse_incoming_order:
                self.get_logger().info(self.parse_order(order))

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

        def _parse_kitting_task(self, kitting_task: KittingTask):
            '''
            Parses a KittingTask object and returns a string representation.

            Args:
                kitting_task (KittingTask): KittingTask object to parse

            Returns:
                str: String representation of the KittingTask object
            '''
            output = 'Type: Kitting\n'
            output += '==========================\n'
            output += f'AGV: {kitting_task.agv_number}\n'
            output += f'Destination: {CompetitionInterface._destinations[kitting_task.destination]}\n'
            output += f'Tray ID: {kitting_task.tray_id}\n'
            output += 'Products:\n'
            output += '==========================\n'

            quadrants = {1: "Quadrant 1: -",
                        2: "Quadrant 2: -",
                        3: "Quadrant 3: -",
                        4: "Quadrant 4: -"}

            for i in range(1, 5):
                product: KittingPart
                for product in kitting_task.parts:
                    if i == product.quadrant:
                        part_color = CompetitionInterface._part_colors[product.part.color].capitalize()
                        part_color_emoji = CompetitionInterface._part_colors_emoji[product.part.color]
                        part_type = CompetitionInterface._part_types[product.part.type].capitalize()
                        quadrants[i] = f'Quadrant {i}: {part_color_emoji} {part_color} {part_type}'
            output += f'\t{quadrants[1]}\n'
            output += f'\t{quadrants[2]}\n'
            output += f'\t{quadrants[3]}\n'
            output += f'\t{quadrants[4]}\n'

            return output

        def _parse_assembly_task(self, assembly_task: AssemblyTask):
            '''
            Parses an AssemblyTask object and returns a string representation.

            Args:
                assembly_task (AssemblyTask): AssemblyTask object to parse

            Returns:
                str: String representation of the AssemblyTask object
            '''
            output = 'Type: Assembly\n'
            output += '==========================\n'
            if len(assembly_task.agv_numbers) == 1:
                output += f'AGV: {assembly_task.agv_number[0]}\n'
            elif len(assembly_task.agv_numbers) == 2:
                output += f'AGV(s): [{assembly_task.agv_numbers[0]}, {assembly_task.agv_numbers[1]}]\n'
            output += f'Assembly station: {self._destinations[assembly_task.station].title()}\n'
            output += 'Products:\n'
            output += '==========================\n'

            product: AssemblyPartMsg
            for product in assembly_task.parts:
                part_color = CompetitionInterface._part_colors[product.part.color].capitalize()
                part_color_emoji = CompetitionInterface._part_colors_emoji[product.part.color]
                part_type = CompetitionInterface._part_types[product.part.type].capitalize()
                assembled_pose_position = product.assembled_pose.pose.position
                assembled_pose_orientation = product.assembled_pose.pose.orientation
                install_direction = product.install_direction
                position = f'x: {assembled_pose_position.x}\n\t\ty: {assembled_pose_position.y}\n\t\tz: {assembled_pose_position.z}'
                orientation = f'x: {assembled_pose_orientation.x}\n\t\ty: {assembled_pose_orientation.y}\n\t\tz: {assembled_pose_orientation.z}\n\t\tw: {assembled_pose_orientation.w}'
                output += f'\tPart: {part_color_emoji} {part_color} {part_type}\n'
                output += '\tPosition:\n'
                output += f'\t\t{position}\n'
                output += '\tOrientation:\n'
                output += f'\t\t{orientation}\n'
                output += f'\tInstall direction: [{install_direction.x}, {install_direction.y}, {install_direction.z}]\n\n'

            return output

        def _parse_combined_task(self, combined_task: CombinedTask):
            '''
            Parses a CombinedTask object and returns a string representation.

            Args:
                combined_task (CombinedTask): CombinedTask object to parse

            Returns:
                str: String representation of the CombinedTask object
            '''

            output = 'Type: Combined\n'
            output += '==========================\n'
            output += f'Assembly station: {self._destinations[combined_task.station].title()}\n'
            output += 'Products:\n'
            output += '==========================\n'

            product: AssemblyPartMsg
            for product in combined_task.parts:
                part_color = CompetitionInterface._part_colors[product.part.color].capitalize()
                part_color_emoji = CompetitionInterface._part_colors_emoji[product.part.color]
                part_type = CompetitionInterface._part_types[product.part.type].capitalize()
                assembled_pose_position = product.assembled_pose.pose.position
                assembled_pose_orientation = product.assembled_pose.pose.orientation
                install_direction = product.install_direction
                position = f'x: {assembled_pose_position.x}\n\t\ty: {assembled_pose_position.y}\n\t\tz: {assembled_pose_position.z}'
                orientation = f'x: {assembled_pose_orientation.x}\n\t\ty: {assembled_pose_orientation.y}\n\t\tz: {assembled_pose_orientation.z}\n\t\tw: {assembled_pose_orientation.w}'
                output += f'\tPart: {part_color_emoji} {part_color} {part_type}\n'
                output += '\tPosition:\n'
                output += f'\t\t{position}\n'
                output += '\tOrientation:\n'
                output += f'\t\t{orientation}\n'
                output += f'\tInstall direction: [{install_direction.x}, {install_direction.y}, {install_direction.z}]\n\n'

            return output

        def parse_order(self, order: Order):
            '''Parse an order message and return a string representation.

            Args:
                order (Order) -- Order message

            Returns:
                String representation of the order message
            '''
            output = '\n\n==========================\n'
            output += f'Received Order: {order.order_id}\n'
            output += f'Priority: {order.order_priority}\n'

            if order.order_type == OrderMsg.KITTING:
                output += self._parse_kitting_task(order.order_task)
            elif order.order_type == OrderMsg.ASSEMBLY:
                output += self._parse_assembly_task(order.order_task)
            elif order.order_type == OrderMsg.COMBINED:
                output += self._parse_combined_task(order.order_task)
            else:
                output += 'Type: Unknown\n'
            return output
        
        def lock_agv_tray(self, num):
            '''
            Lock the tray of an AGV and parts on the tray. This will prevent tray and parts from moving during transport.

            Args:
                num (int):  AGV number

            Raises:
                KeyboardInterrupt: Exception raised when the user presses Ctrl+C
            '''        
            
            # Create a client to send a request to the `/ariac/agv{num}_lock_tray` service
            tray_locker = self.create_client(
                Trigger,
                f'/ariac/agv{num}_lock_tray'
            )

            # Build the request
            request = Trigger.Request()
            # Send the request
            future = tray_locker.call_async(request)

            # Wait for the response
            try:
                rclpy.spin_until_future_complete(self, future)
            except KeyboardInterrupt as kb_error:
                raise KeyboardInterrupt from kb_error

            # Check the response
            if future.result().success:
                self.get_logger().info(f'Locked AGV{num}\'s tray')
            else:
                self.get_logger().warn('Unable to lock tray')

        def move_agv_to_station(self, num, station):
            '''
            Move an AGV to an assembly station.

            Args:
                num (int): AGV number
                station (int): Assembly station number

            Raises:
                KeyboardInterrupt: Exception raised when the user presses Ctrl+C
            '''        
            
            # Create a client to send a request to the `/ariac/move_agv` service.
            mover = self.create_client(
                MoveAGV,
                f'/ariac/move_agv{num}')

            # Create a request object.
            request = MoveAGV.Request()

            # Set the request location.
            if station in [AssemblyTaskMsg.AS1, AssemblyTaskMsg.AS3]:
                request.location = MoveAGV.Request.ASSEMBLY_FRONT
            else:
                request.location = MoveAGV.Request.ASSEMBLY_BACK

            # Send the request.
            future = mover.call_async(request)

            # Wait for the server to respond.
            try:
                rclpy.spin_until_future_complete(self, future)
            except KeyboardInterrupt as kb_error:
                raise KeyboardInterrupt from kb_error

            # Check the result of the service call.
            if future.result().success:
                self.get_logger().info(f'Moved AGV{num} to {self._stations[station]}')
            else:
                self.get_logger().warn(future.result().message)




Code Explained
^^^^^^^^^^^^^^^^^^^^^^^

- Imports

    - :inline-python:`from ariac_msgs.srv import MoveAGV`: Service for moving an AGV.


- Instance Methods

    - :inline-python:`move_agv_to_station(self, num, station)`: Method to move an AGV to a station. This method creates a client to the ``/ariac/move_agv{num}`` service and calls it. The AGV number and station are passed as arguments to the method.
    - :inline-python:`lock_agv_tray(self, num)`: Method to lock the tray of an AGV. This method creates a client to the ``/ariac/agv{num}_lock_tray`` service and calls it. The AGV number is passed as an argument to the method.


Create the Executable
--------------------------------

.. code-block:: python
    :caption: tutorial_5.py
    
    #!/usr/bin/env python3

    import rclpy
    from ariac_msgs.msg import Order as OrderMsg
    from ariac_tutorials.competition_interface import CompetitionInterface


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

Code Explained
^^^^^^^^^^^^^^^^^^^^^^^

This executable does the following:

    - Create an instance of the class :inline-python:`CompetitionInterface` as a ROS node.
    - Start the competition.
    - Iterate through the list of orders and retrieve orders with assembly tasks.

        - See :ref:`Tutorial 4 <Tutorial 4>` for more information on retrieving orders.

    - Retrieve AGVs for the assembly tasks.
    - Lock the tray of the AGVs.
    - Move the AGVs to the assembly station.


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

