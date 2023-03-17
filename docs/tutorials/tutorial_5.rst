
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
    │   └── competition_interface.py
    └── src
        ├── start_competition.py        (from tutorial 1)
        ├── read_break_beam_sensor.py   (from tutorial 2)
        ├── read_advanced_camera.py     (from tutorial 3)
        ├── read_orders.py              (from tutorial 4)
        └── read_orders.py              (new)





Competition Interface
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
        AssemblyPart as AssemblyPartMsg,
        AssemblyTask as AssemblyTaskMsg,
        AGVStatus as AGVStatusMsg
    )

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
                self.competition_state_cb,
                10)

            # Store the state of the competition
            self._competition_state: CompetitionStateMsg = None

            # Subscriber to the order topic
            self._orders_sub = self.create_subscription(OrderMsg, '/ariac/orders', self.orders_cb, 10)
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
        if self._parse_incoming_order:
            self.get_logger().info(self.parse_order(order))

    def parse_kitting_task(self, kitting_task: KittingTask):
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

    def parse_assembly_task(self, assembly_task: AssemblyTask):
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

    def parse_combined_task(self, combined_task: CombinedTask):
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
            output += self.parse_kitting_task(order.order_task)
        elif order.order_type == OrderMsg.ASSEMBLY:
            output += self.parse_assembly_task(order.order_task)
        elif order.order_type == OrderMsg.COMBINED:
            output += self.parse_combined_task(order.order_task)
        else:
            output += 'Type: Unknown\n'
        return output




Contents of the competition interface specific to this tutorial are described as follows:

    - Multiple messages from the package ``ariac_msgs`` are imported to store the content of messages published to the topic ``/ariac/orders``. 
    - Data classes: Multiple data classes are used to store the content of messages published to the topic ``/ariac/orders``. Best practices for creating data classes are described in the `Python documentation <https://docs.python.org/3/library/dataclasses.html>`_.

    - ``__init__()``: 

        - ``_orders_sub``: This is the subscriber to the topic ``/ariac/orders``. The callback function is ``orders_cb()``. 
        - ``_orders``: This is a list of orders that have been received. It is initialized to an empty list.
        - ``_parse_incoming_order``: This is a boolean that determines whether the competition interface should parse (display on the standard output) incoming orders.
 
    - ``orders_cb()``: This is the callback method for the subscriber ``_orders_sub``. It is called whenever a new message is published to the topic ``/ariac/orders``. The content of the message is stored in the list ``_orders``. If ``_parse_incoming_order`` is ``True``, the content of the message is parsed and displayed on the standard output.
    - ``parse_order()``: This method parses the content of an order message and returns a string representation. It is called by ``orders_cb()`` if ``_parse_incoming_order`` is ``True``. This method calls the methods ``parse_kitting_task()``, ``parse_assembly_task()``, and ``parse_combined_task()`` depending on the type of the order.




Configure the Executable
--------------------------------

To test this tutorial, create a new file ``read_orders.py`` in ``competition_tutorials/src``:

.. code-block:: bash

    cd ~/ariac_ws/src/competition_tutorials/src
    touch read_orders.py
    chmod +x read_orders.py


Copy the following code in the file ``read_orders.py``:


.. code-block:: python
    :caption: read_orders.py
    
    #!/usr/bin/env python3

    import rclpy
    from competition_tutorials.competition_interface import CompetitionInterface

    def main(args=None):
        rclpy.init(args=args)
        interface = CompetitionInterface()
        interface.start_competition()
        # The following line enables order displays in the terminal.
        interface.parse_incoming_order = True

        while rclpy.ok():
            try:
                rclpy.spin_once(interface)
            except KeyboardInterrupt:
                break

        interface.destroy_node()
        rclpy.shutdown()

    if __name__ == '__main__':
        main()

In the main function we set the variable ``parse_incoming_order`` to ``True``. This will cause the competition interface to parse incoming orders and display them on the standard output. To disable this feature, set ``parse_incoming_order`` to ``False``.


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
    ros2 run competition_tutorials read_orders.py


The node will wait until the competition is ready. In a second terminal, run the following:

.. code-block:: bash
    :caption: Terminal 2

    cd ~/ariac_ws
    . install/setup.bash
    ros2 launch ariac_gazebo ariac.launch.py trial_name:=tutorial


Once the environment is loaded and the competition state is ready, the interface node running in Terminal 1 will start the competition and published orders will be displayed on the standard output in Terminal 1.


Outputs
--------------------------------

Terminal outputs of tutorial 4 displaying received orders are provided below.

.. code-block:: bash
    :caption: Terminal outputs
    
    ==========================
    Received Order: 2IZJP127
    Priority: False
    Type: Assembly
    ==========================
    AGV(s): [1, 2]
    Assembly station: Front Assembly Station
    Products:
    ==========================
        Part: 🟥 Red Regulator
        Position:
            x: 0.175
            y: -0.223
            z: 0.215
        Orientation:
            x: 0.5
            y: -0.4999999999999999
            z: -0.5
            w: 0.5000000000000001
        Install direction: [0.0, 0.0, -1.0]

        Part: 🟥 Red Battery
        Position:
            x: -0.15
            y: 0.035
            z: 0.043
        Orientation:
            x: 0.0
            y: 0.0
            z: 0.7071067811865475
            w: 0.7071067811865476
        Install direction: [0.0, 1.0, 0.0]

        Part: 🟥 Red Pump
        Position:
            x: 0.14
            y: 0.0
            z: 0.02
        Orientation:
            x: 0.0
            y: 0.0
            z: -0.7071067811865475
            w: 0.7071067811865476
        Install direction: [0.0, 0.0, -1.0]

        Part: 🟥 Red Sensor
        Position:
            x: -0.1
            y: 0.395
            z: 0.045
        Orientation:
            x: 0.0
            y: 0.0
            z: -0.7071067811865475
            w: 0.7071067811865476
        Install direction: [0.0, -1.0, 0.0]


    [INFO] [1679041253.912411883] [competition_interface]: 

    ==========================
    Received Order: 2IZJP320
    Priority: False
    Type: Combined
    ==========================
    Assembly station: Warehouse
    Products:
    ==========================
        Part: 🟧 Orange Regulator
        Position:
            x: 0.175
            y: -0.223
            z: 0.215
        Orientation:
            x: 0.5
            y: -0.4999999999999999
            z: -0.5
            w: 0.5000000000000001
        Install direction: [0.0, 0.0, -1.0]

        Part: 🟧 Orange Battery
        Position:
            x: -0.15
            y: 0.035
            z: 0.043
        Orientation:
            x: 0.0
            y: 0.0
            z: 0.7071067811865475
            w: 0.7071067811865476
        Install direction: [0.0, 1.0, 0.0]

        Part: 🟧 Orange Pump
        Position:
            x: 0.14
            y: 0.0
            z: 0.02
        Orientation:
            x: 0.0
            y: 0.0
            z: -0.7071067811865475
            w: 0.7071067811865476
        Install direction: [0.0, 0.0, -1.0]

        Part: 🟧 Orange Sensor
        Position:
            x: -0.1
            y: 0.395
            z: 0.045
        Orientation:
            x: 0.0
            y: 0.0
            z: -0.7071067811865475
            w: 0.7071067811865476
        Install direction: [0.0, -1.0, 0.0]


    [INFO] [1679041253.913566162] [competition_interface]: 

    ==========================
    Received Order: MMB30H56
    Priority: False
    Type: Kitting
    ==========================
    AGV: 1
    Destination: warehouse
    Tray ID: 3
    Products:
    ==========================
        Quadrant 1: 🟪 Purple Pump
        Quadrant 2: -
        Quadrant 3: 🟦 Blue Battery
        Quadrant 4: -

    [INFO] [1679041259.750922649] [competition_interface]: 

    ==========================
    Received Order: MMB30H57
    Priority: False
    Type: Kitting
    ==========================
    AGV: 2
    Destination: warehouse
    Tray ID: 5
    Products:
    ==========================
        Quadrant 1: -
        Quadrant 2: 🟧 Orange Regulator
        Quadrant 3: -
        Quadrant 4: -

    [INFO] [1679041268.581512935] [competition_interface]: 

    ==========================
    Received Order: MMB30H58
    Priority: False
    Type: Kitting
    ==========================
    AGV: 3
    Destination: warehouse
    Tray ID: 8
    Products:
    ==========================
        Quadrant 1: -
        Quadrant 2: -
        Quadrant 3: -
        Quadrant 4: 🟩 Green Sensor
