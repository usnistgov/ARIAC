
.. _TUTORIAL_6:

.. only:: builder_html or readthedocs

.. role:: inline-python(code)
    :language: python

.. role:: inline-file(file)

.. role:: inline-tutorial(file)

.. role:: inline-bash(code)
    :language: bash

=========================================================
Tutorial 6: Activate/Deativate a Gripper
=========================================================

.. admonition:: Source Code for Tutorial 6
  :class: attention
  :name: tutorial_6
  
  `https://github.com/jaybrecht/ariac_tutorials/tree/tutorial_6 <https://github.com/jaybrecht/ariac_tutorials/tree/tutorial_6>`_ 

  .. code-block:: bash
    
        cd ~/ariac_ws/ariac_tutorials
        git checkout tutorial_6


This tutorial covers the following steps:

  - Parse received orders to identify assembly tasks, 
  - identify the AGVs that need to be moved to the assembly station,
  - Lock the tray and parts so that they do not fall off the AGV,
  - Move the AGV to the assembly station.

The final state of the package :inline-file:`ariac_tutorials` for :inline-tutorial:`tutorial 6` is as follows:

.. code-block:: text
    :emphasize-lines: 16
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
        ├── tutorial_5.py
        └── tutorial_6.py




Competition Interface
--------------------------------

The competition interface used in this tutorial is shown in :numref:`competitioninterface-tutorial6`.

.. code-block:: python
    :caption: competition_interface.py
    :name: competitioninterface-tutorial6
    :linenos:

    import rclpy
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.time import Duration

    from ariac_msgs.msg import (
        CompetitionState as CompetitionStateMsg,
        Part as PartMsg,
        Order as OrderMsg,
        AssemblyPart as AssemblyPartMsg,
        AssemblyTask as AssemblyTaskMsg,
        AGVStatus as AGVStatusMsg,
        VacuumGripperState,
    )

    from rclpy.qos import qos_profile_sensor_data
    from std_srvs.srv import Trigger
    from ariac_msgs.srv import MoveAGV
    from ariac_msgs.srv import VacuumGripperControl

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
        
        _gripper_states = {
            True: 'enabled',
            False: 'disabled'
        }
        '''Dictionary for converting VacuumGripperState constants to strings'''

        _part_colors = {
            PartMsg.RED: 'red',
            PartMsg.BLUE: 'blue',
            PartMsg.GREEN: 'green',
            PartMsg.ORANGE: 'orange',
            PartMsg.PURPLE: 'purple',
        }
        '''Dictionary for converting Part color constants to strings'''

        _part_colors_emoji = {
            PartMsg.RED: '🟥',
            PartMsg.BLUE: '🟦',
            PartMsg.GREEN: '🟩',
            PartMsg.ORANGE: '🟧',
            PartMsg.PURPLE: '🟪',
        }
        '''Dictionary for converting Part color constants to emojis'''


        _part_types = {
            PartMsg.BATTERY: 'battery',
            PartMsg.PUMP: 'pump',
            PartMsg.REGULATOR: 'regulator',
            PartMsg.SENSOR: 'sensor',
        }
        '''Dictionary for converting Part type constants to strings'''

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
        '''Dictionary for converting AGVStatus constants to strings'''

        _stations = {
            AssemblyTaskMsg.AS1: "assembly station 1",
            AssemblyTaskMsg.AS2: "assembly station 2",
            AssemblyTaskMsg.AS3: "assembly station 3",
            AssemblyTaskMsg.AS4: "assembly station 4",
        }
        '''Dictionary for converting AssemblyTask constants to strings'''

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
            
            # Subscriber to the floor gripper state topic
            self._floor_robot_gripper_state_sub = self.create_subscription(
                VacuumGripperState,
                '/ariac/floor_robot_gripper_state',
                self._floor_robot_gripper_state_cb,
                qos_profile_sensor_data)

            # Service client for turning on/off the vacuum gripper on the floor robot
            self._floor_gripper_enable = self.create_client(
                VacuumGripperControl,
                "/ariac/floor_robot_enable_gripper")

            # Attribute to store the current state of the floor robot gripper
            self._floor_robot_gripper_state = VacuumGripperState()

        @property
        def parse_incoming_order(self):
            '''Property for the parse_incoming_order flag.'''
            return self._parse_incoming_order

        @parse_incoming_order.setter
        def parse_incoming_order(self, value: bool):
            self._parse_incoming_order = value
            
        def _floor_robot_gripper_state_cb(self, msg: VacuumGripperState):
            '''
            Callback for the floor robot gripper state topic.

            Args:
                msg (VacuumGripperState): VacuumGripperState message
            '''        
            self._floor_robot_gripper_state = msg
            

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
                
        def set_floor_robot_gripper_state(self, state):
            '''Set the gripper state of the floor robot.

            Arguments:
                state -- True to enable, False to disable

            Raises:
                KeyboardInterrupt: Exception raised when the user presses Ctrl+C
            '''
            if self._floor_robot_gripper_state.enabled == state:
                self.get_logger().warn(f'Gripper is already {self._gripper_states[state]}')
                return

            request = VacuumGripperControl.Request()
            request.enable = state

            future = self._floor_gripper_enable.call_async(request)

            try:
                rclpy.spin_until_future_complete(self, future)
            except KeyboardInterrupt as kb_error:
                raise KeyboardInterrupt from kb_error

            if future.result().success:
                self.get_logger().info(f'Changed gripper state to {self._gripper_states[state]}')
            else:
                self.get_logger().warn('Unable to change gripper state')

        def wait(self, duration):
            '''Wait for a specified duration.

            Arguments:
                duration -- Duration to wait in seconds

            Raises:
                KeyboardInterrupt: Exception raised when the user presses Ctrl+C
            '''
            start = self.get_clock().now()

            while self.get_clock().now() <= start + Duration(seconds=duration):
                try:
                    rclpy.spin_once(self)
                except KeyboardInterrupt as kb_error:
                    raise KeyboardInterrupt from kb_error


Code Explained
^^^^^^^^^^^^^^^^^^^^^^^

- Imports

    - ``from rclpy.time import Duration``: A ``wait()`` function is implemented using the ``Duration`` class. The wait function is used to wait for a specified duration while the gripper state is being changed.
    - ``from rclpy.qos import qos_profile_sensor_data``: ROS 2 Quality of Service API. This is used to set the QoS profile for the floor robot gripper state subscriber.
    - ``from ariac_msgs.msg import VacuumGripperState``: Message type for the vacuum gripper state. 
    - ``from ariac_msgs.srv import VacuumGripperControl``: Service type for controlling the vacuum gripper. 

- Class Variables

    - ``_gripper_states``: A dictionary for converting the ``VacuumGripperState`` constants to strings. This is used for logging the gripper state.

- Instance Variables

    - ``_floor_robot_gripper_state_sub``: Subscriber to the floor robot gripper state topic.
    - ``_floor_gripper_enable``: Service client for turning on/off the vacuum gripper on the floor robot.
    - ``_floor_robot_gripper_state``: Attribute to store the current state of the floor robot gripper.


- Instance Methods

    - ``_floor_robot_gripper_state_cb(self, msg: VacuumGripperState)``: Callback for the topic ``/ariac/floor_robot_gripper_state``. This is used to store the current state of the floor robot gripper.
    - ``set_floor_robot_gripper_state(self, state)``: Function to set the gripper state of the floor robot. This function calls the ROS service to change the gripper state.
    - ``wait(self, duration)``: Function to wait for a specified duration. This function is used to wait for the gripper state to change.


Create the Executable
--------------------------------

.. code-block:: python
    :caption: tutorial_6.py
    
    #!/usr/bin/env python3


    import rclpy
    from ariac_tutorials.competition_interface import CompetitionInterface

    def main(args=None):
        rclpy.init(args=args)
        interface = CompetitionInterface()
        interface.start_competition()

        while rclpy.ok():
            try:
                interface.set_floor_robot_gripper_state(True)
                interface.wait(3)
                interface.set_floor_robot_gripper_state(False)
                interface.wait(3)
            except KeyboardInterrupt:
                break

        interface.destroy_node()
        rclpy.shutdown()

    if __name__ == '__main__':
        main()

Code Explained
^^^^^^^^^^^^^^^^^^^^^^^

This executable does the following:

    - Create an instance of the class :inline-python:`CompetitionInterface` as a ROS node.
    - Start the competition.
    - In a while loop:
        
            - Enable the gripper of the floor robot.
            - Wait for 3 seconds.
            - Disable the gripper of the floor robot.
            - Wait for 3 seconds.



Run the Executable
--------------------------------

- In *terminal 1*, run the following commands:


    .. code-block:: bash

        cd ~/ariac_ws
        colcon build
        . install/setup.bash
        ros2 run ariac_tutorials tutorial_6.py


    The node will wait until the competition is ready.


- In *terminal 2*, run the following commands:

    .. code-block:: bash

        cd ~/ariac_ws
        . install/setup.bash
        ros2 launch ariac_gazebo ariac.launch.py competitor_pkg:=ariac_tutorials trial_name:=tutorials



Outputs
--------------------------------


.. code-block:: console
    :caption: Terminal outputs
    :class: no-copybutton
    
    [INFO] [1679048497.138846958] [competition_interface]: Waiting for competition to be ready
    [INFO] [1679048497.139894604] [competition_interface]: Competition state is: ready
    [INFO] [1679048497.140293729] [competition_interface]: Competition is ready. Starting...
    [INFO] [1679048497.142822117] [competition_interface]: Started competition.
    [INFO] [1679048497.145127615] [competition_interface]: Changed gripper state to enabled
    [INFO] [1679048501.986702439] [competition_interface]: Changed gripper state to disabled
    [INFO] [1679048507.031545831] [competition_interface]: Changed gripper state to enabled


