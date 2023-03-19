

.. _TUTORIAL_3:

.. only:: builder_html or readthedocs

.. role:: inline-python(code)
    :language: python

.. role:: inline-file(file)

.. role:: inline-tutorial(file)

.. role:: inline-bash(code)
    :language: bash

=========================================================
Tutorial 3: Reading Data from an Advanced Logical Camera
=========================================================

.. admonition:: Source Code for Tutorial 4
  :class: attention
  :name: tutorial_4
  
  `https://github.com/jaybrecht/ariac_tutorials/tree/tutorial_4 <https://github.com/jaybrecht/ariac_tutorials/tree/tutorial_4>`_ 

  .. code-block:: bash
    
        cd ~/ariac_ws/ariac_tutorials
        git checkout tutorial_4


This tutorial covers the following steps:

  - Receive order messages, 
  - Store each order internally as an instance of a class,
  - Display each order on the standard output.

The final state of the package :inline-file:`ariac_tutorials` for :inline-tutorial:`tutorial 4` is as follows:

.. code-block:: text
    :emphasize-lines: 11
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
        └── tutorial_4.py




Competition Interface
--------------------------------

The competition interface used in this tutorial is shown in :numref:`competitioninterface-tutorial4`.

.. code-block:: python
    :caption: competition_interface.py
    :name: competitioninterface-tutorial4
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


Code Explained
^^^^^^^^^^^^^^^^^^^^^^^

- Imports

    - :inline-python:`ariac_msgs.msg`: Messages from the package :inline-file:`ariac_msgs`.

        - :inline-python:`Order`: Message that contains the order information.

    - :inline-python:`ariac_tutorials.utils`: Module which contains reusable functions and classes. 
  
- Instance Variables

    - :inline-python:`self._orders_sub`: ROS subscriber to the topic ``/ariac/orders``
    - :inline-python:`self._orders`: List of orders.
    - :inline-python:`self._parse_incoming_order`: Flag for for logging an order in the terminal. If the flag is set to :inline-python:`True`, the order is logged in the terminal. If the flag is set to :inline-python:`False`, the order is not logged in the terminal.


- Instance Methods

    - :inline-python:`_orders_cb(self, msg: OrderMsg)`: Callback method for the subscriber to the topic ``/ariac/orders``. It parses the order and stores it in the list of orders :inline-python:`self._orders`
    - :inline-python:`parse_order(self, order: Order)`: Parses an order message and returns a string representation. This method calls the appropriate parsing method  based on the type of the order.

        - :inline-python:`_parse_kitting_task(self, kitting_task: KittingTask)`: Parses a KittingTask object and returns a string representation.
        - :inline-python:`_parse_assembly_task(self, assembly_task: AssemblyTask)`: Parses an AssemblyTask object and returns a string representation.
        - :inline-python:`_parse_combined_task(self, combined_task: CombinedTask)`: Parses a CombinedTask object and returns a string representation.



Create the Executable
--------------------------------

.. code-block:: python
    :caption: tutorial_4.py
    
    #!/usr/bin/env python3

    import rclpy
    from competition_tutorials.competition_interface import CompetitionInterface

    def main(args=None):
        rclpy.init(args=args)
        interface = CompetitionInterface()
        interface.start_competition()
        # The following line enables order displays in the terminal.
        # To disable order displays, set parse_incoming_order to False.
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

Code Explained
^^^^^^^^^^^^^^^^^^^^^^^

This executable does the following:

    - Creates an instance of the class :inline-python:`CompetitionInterface` as a ROS node.
    - Starts the competition.
    - Logs  each published message to the terminal.



Run the Executable
--------------------------------

- In *terminal 1*, run the following commands:


    .. code-block:: bash

        cd ~/ariac_ws
        colcon build
        . install/setup.bash
        ros2 run ariac_tutorials tutorial_4.py


    The node will wait until the competition is ready.


- In *terminal 2*, run the following commands:

    .. code-block:: bash

        cd ~/ariac_ws
        . install/setup.bash
        ros2 launch ariac_gazebo ariac.launch.py competitor_pkg:=ariac_tutorials trial_name:=tutorials



Once the environment is loaded and the competition state is ready, the interface node running in *terminal 1* will start the competition and published orders will be displayed on the standard output.


Outputs
--------------------------------


.. code-block:: console
    :caption: Terminal outputs
    :class: no-copybutton
    
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
