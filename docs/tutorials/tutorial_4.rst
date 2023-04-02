.. _TUTORIAL4:

*************************
Tutorial 4: Read an Order
*************************

.. admonition:: Tutorial 4
  :class: attention
  :name: tutorial_4

  - **Prerequisites:** :ref:`Introduction to Tutorials <TUTORIALS>` and :ref:`Tutorial 3 <TUTORIAL3>`
  - **Source Code**: `https://github.com/jaybrecht/ariac_tutorials/tree/tutorial_4 <https://github.com/jaybrecht/ariac_tutorials/tree/tutorial_4>`_ 
  - **Switch Branch**:

    .. code-block:: bash
        
            cd ~/ariac_ws/src/ariac_tutorials
            git switch tutorial_4


This tutorial shows how to read each order published by the ARIAC manager. The following steps are performed:

  - Receive order messages, 
  - Store each order internally as an instance of a class,
  - Display each order on the standard output.

Package Structure
=================

Updates and additions that are specific to :tuto:`Tutorial 4`  are highlighted in the tree below.


.. code-block:: text
    :emphasize-lines: 2, 9, 14
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
        └── tutorial_4.py


Updated/Created Files
=====================

Competition Interface
---------------------

The competition interface for :tuto:`Tutorial 4` is shown in :numref:`competitioninterface-tutorial4`.

.. code-block:: python
    :caption: :file:`competition_interface.py`
    :name: competitioninterface-tutorial4
    :emphasize-lines: 12-15, 25-29, 78-92, 141-152, 154-156, 166-168, 170-172, 174-182, 301-337, 339-383, 385-426, 428-449
    :linenos:

    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.parameter import Parameter

    from ariac_msgs.msg import (
        CompetitionState as CompetitionStateMsg,
        BreakBeamStatus as BreakBeamStatusMsg,
        AdvancedLogicalCameraImage as AdvancedLogicalCameraImageMsg,
        Part as PartMsg,
        PartPose as PartPoseMsg,
        Order as OrderMsg,
        AssemblyPart as AssemblyPartMsg,
        AGVStatus as AGVStatusMsg,
        AssemblyTask as AssemblyTaskMsg,
    )

    from std_srvs.srv import Trigger

    from ariac_tutorials.utils import (
        multiply_pose,
        rpy_from_quaternion,
        rad_to_deg_str,
        AdvancedLogicalCameraImage,
        Order,
        KittingTask,
        CombinedTask,
        AssemblyTask,
        KittingPart,
    )


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
        
        _destinations = {
            AGVStatusMsg.KITTING: 'kitting station',
            AGVStatusMsg.ASSEMBLY_FRONT: 'front assembly station',
            AGVStatusMsg.ASSEMBLY_BACK: 'back assembly station',
            AGVStatusMsg.WAREHOUSE: 'warehouse',
        }
        '''Dictionary for converting AGVDestination constants to strings'''

        _stations = {
            AssemblyTaskMsg.AS1: 'assembly station 1',
            AssemblyTaskMsg.AS2: 'assembly station 2',
            AssemblyTaskMsg.AS3: 'assembly station 3',
            AssemblyTaskMsg.AS4: 'assembly station 4',
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

            # Subscriber to the break beam status topic
            self._break_beam0_sub = self.create_subscription(
                BreakBeamStatusMsg,
                '/ariac/sensors/breakbeam_0/status',
                self._breakbeam0_cb,
                qos_profile_sensor_data)
            
            # Store the number of parts that crossed the beam
            self._conveyor_part_count = 0

            # Store whether the beam is broken
            self._object_detected = False
            
            # Subscriber to the logical camera topic
            self._advanced_camera0_sub = self.create_subscription(
                AdvancedLogicalCameraImageMsg,
                '/ariac/sensors/advanced_camera_0/image',
                self._advanced_camera0_cb,
                qos_profile_sensor_data)
            
            # Store each camera image as an AdvancedLogicalCameraImage object
            self._camera_image: AdvancedLogicalCameraImage = None
            
            # Subscriber to the order topic
            self.orders_sub = self.create_subscription(
                OrderMsg,
                '/ariac/orders',
                self._orders_cb,
                10)
            
            # Flag for parsing incoming orders
            self._parse_incoming_order = False

            # List of orders
            self._orders = []

        @property
        def orders(self):
            return self._orders
        
        @property
        def camera_image(self):
            return self._camera_image

        @property
        def conveyor_part_count(self):
            return self._conveyor_part_count
        
        @property
        def parse_incoming_order(self):
            return self._parse_incoming_order
        
        @parse_incoming_order.setter
        def parse_incoming_order(self, value):
            self._parse_incoming_order = value
        
        def _orders_cb(self, msg: Order):
            '''Callback for the topic /ariac/orders
            Arguments:
                msg -- Order message
            '''
            order = Order(msg)
            self._orders.append(order)
            if self._parse_incoming_order:
                self.get_logger().info(self._parse_order(order))

        def _advanced_camera0_cb(self, msg: AdvancedLogicalCameraImageMsg):
            '''Callback for the topic /ariac/sensors/advanced_camera_0/image

            Arguments:
                msg -- AdvancedLogicalCameraImage message
            '''
            self._camera_image = AdvancedLogicalCameraImage(msg.part_poses,
                                                            msg.tray_poses,
                                                            msg.sensor_pose)

        def _breakbeam0_cb(self, msg: BreakBeamStatusMsg):
            '''Callback for the topic /ariac/sensors/breakbeam_0/status

            Arguments:
                msg -- BreakBeamStatusMsg message
            '''
            if not self._object_detected and msg.object_detected:
                self._conveyor_part_count += 1

            self._object_detected = msg.object_detected

        def _competition_state_cb(self, msg: CompetitionStateMsg):
            '''Callback for the topic /ariac/competition_state
            Arguments:
                msg -- CompetitionState message
            '''
            # Log if competition state has changed
            if self._competition_state != msg.competition_state:
                state = CompetitionInterface._competition_states[msg.competition_state]
                self.get_logger().info(f'Competition state is: {state}', throttle_duration_sec=1.0)
            
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

            # Check if service is available
            if not self._start_competition_client.wait_for_service(timeout_sec=3.0):
                self.get_logger().error('Service \'/ariac/start_competition\' is not available.')
                return

            # Create trigger request and call starter service
            request = Trigger.Request()
            future = self._start_competition_client.call_async(request)

            # Wait until the service call is completed
            rclpy.spin_until_future_complete(self, future)

            if future.result().success:
                self.get_logger().info('Started competition.')
            else:
                self.get_logger().warn('Unable to start competition')
                
        def parse_advanced_camera_image(self, image: AdvancedLogicalCameraImage) -> str:
            '''
            Parse an AdvancedLogicalCameraImage message and return a string representation.
            '''
            
            if len(image._part_poses) == 0:
                return 'No parts detected'

            output = '\n\n'
            for i, part_pose in enumerate(image._part_poses):
                part_pose: PartPoseMsg
                output += '==========================\n'
                part_color = CompetitionInterface._part_colors[part_pose.part.color].capitalize()
                part_color_emoji = CompetitionInterface._part_colors_emoji[part_pose.part.color]
                part_type = CompetitionInterface._part_types[part_pose.part.type].capitalize()
                output += f'Part {i+1}: {part_color_emoji} {part_color} {part_type}\n'
                output += '--------------------------\n'
                output += 'Camera Frame\n'
                output += '--------------------------\n'
                
                output += '  Position:\n'
                output += f'    x: {part_pose.pose.position.x:.3f} (m)\n'
                output += f'    y: {part_pose.pose.position.y:.3f} (m)\n'
                output += f'    z: {part_pose.pose.position.z:.3f} (m)\n'

                roll, pitch, yaw = rpy_from_quaternion(part_pose.pose.orientation)
                output += '  Orientation:\n'
                output += f'    roll: {rad_to_deg_str(roll)}\n'
                output += f'    pitch: {rad_to_deg_str(pitch)}\n'
                output += f'    yaw: {rad_to_deg_str(yaw)}\n'
                
                part_world_pose = multiply_pose(image._sensor_pose, part_pose.pose)
                output += '--------------------------\n'
                output += 'World Frame\n'
                output += '--------------------------\n'

                output += '  Position:\n'
                output += f'    x: {part_world_pose.position.x:.3f} (m)\n'
                output += f'    y: {part_world_pose.position.y:.3f} (m)\n'
                output += f'    z: {part_world_pose.position.z:.3f} (m)\n'

                roll, pitch, yaw = rpy_from_quaternion(part_world_pose.orientation)
                output += '  Orientation:\n'
                output += f'    roll: {rad_to_deg_str(roll)}\n'
                output += f'    pitch: {rad_to_deg_str(pitch)}\n'
                output += f'    yaw: {rad_to_deg_str(yaw)}\n'

                output += '==========================\n\n'

            return output
        
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
            output += f'Station: {self._stations[assembly_task.station].title()}\n'
            output += 'Products:\n'
            output += '==========================\n'

            product: AssemblyPartMsg
            for product in assembly_task.parts:
                part_color = CompetitionInterface._part_colors[product.part.color].capitalize()
                part_color_emoji = CompetitionInterface._part_colors_emoji[product.part.color]
                part_type = CompetitionInterface._part_types[product.part.type].capitalize()

                output += f'Part: {part_color_emoji} {part_color} {part_type}\n'

                output += '  Position:\n'
                output += f'    x: {product.assembled_pose.pose.position.x:.3f} (m)\n'
                output += f'    y: {product.assembled_pose.pose.position.y:.3f} (m)\n'
                output += f'    z: {product.assembled_pose.pose.position.z:.3f} (m)\n'

                roll, pitch, yaw = rpy_from_quaternion(product.assembled_pose.pose.orientation)
                output += '  Orientation:\n'
                output += f'    roll: {rad_to_deg_str(roll)}\n'
                output += f'    pitch: {rad_to_deg_str(pitch)}\n'
                output += f'    yaw: {rad_to_deg_str(yaw)}\n'

                output += f'  Install direction:\n'
                output += f'    x: {product.install_direction.x:.1f}\n'
                output += f'    y: {product.install_direction.y:.1f}\n'
                output += f'    z: {product.install_direction.z:.1f}\n'

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
            output += f'Station: {self._stations[combined_task.station].title()}\n'
            output += 'Products:\n'
            output += '==========================\n'

            product: AssemblyPartMsg
            for product in combined_task.parts:
                part_color = CompetitionInterface._part_colors[product.part.color].capitalize()
                part_color_emoji = CompetitionInterface._part_colors_emoji[product.part.color]
                part_type = CompetitionInterface._part_types[product.part.type].capitalize()

                output += f'Part: {part_color_emoji} {part_color} {part_type}\n'

                output += '  Position:\n'
                output += f'    x: {product.assembled_pose.pose.position.x:.3f} (m)\n'
                output += f'    y: {product.assembled_pose.pose.position.y:.3f} (m)\n'
                output += f'    z: {product.assembled_pose.pose.position.z:.3f} (m)\n'

                roll, pitch, yaw = rpy_from_quaternion(product.assembled_pose.pose.orientation)
                output += '  Orientation:\n'
                output += f'    roll: {rad_to_deg_str(roll)}\n'
                output += f'    pitch: {rad_to_deg_str(pitch)}\n'
                output += f'    yaw: {rad_to_deg_str(yaw)}\n'

                output += f'  Install direction:\n'
                output += f'    x: {product.install_direction.x:.1f}\n'
                output += f'    y: {product.install_direction.y:.1f}\n'
                output += f'    z: {product.install_direction.z:.1f}\n'

            return output

        def _parse_order(self, order: Order):
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



Code Explanation
^^^^^^^^^^^^^^^^^

The competition interface from :ref:`Tutorial 3 <TUTORIAL3>` was augmented with the components described below.

- Imports

    - :python:`Order`: Message class that contains the order information (:term:`ariac_msgs/msg/Order`).
    - :python:`AssemblyPart`: Message class that contains the assembly part information (:term:`ariac_msgs/msg/AssemblyPart`).
    - :python:`AssemblyTask`: Message class that contains the assembly task information (:term:`ariac_msgs/msg/AssemblyTask`).
    - :python:`AGVStatus`: Message class that contains the AGV status information ( :term:`ariac_msgs/msg/AGVStatus`).

- Class Variables

    - :python:`_destinations` is a dictionary that maps the integer values of the AGV destination to their string representations.
    - :python:`_stations` is a dictionary that maps the integer values of the assembly stations to their string representations.

- Instance Variables

    - :python:`_orders_sub`: ROS subscriber to the topic :topic:`/ariac/orders`.

    - :python:`self._parse_incoming_order`: Flag for logging an order in the terminal. If the flag is set to :python:`True`, the order is logged in the terminal. If the flag is set to :python:`False`, the order is not logged in the terminal.

    - :python:`self._orders`: List of orders. Each order announced by the competition interface is stored in this list.

- Instance Methods

    - :python:`orders(self)`: Getter for the list of orders :python:`self._orders`

    - :python:`parse_incoming_order(self)` Getter for the flag :python:`self._parse_incoming_order`

    - :python:`parse_incoming_order(self, value)` Setter for the flag :python:`self._parse_incoming_order`

    - :python:`_orders_cb(self, msg: OrderMsg)`: Callback method for the subscriber to the topic :topic:`/ariac/orders`. It parses the order and stores it in the list of orders :python:`self._orders`

    - :python:`_parse_order(self, order: Order)`: Parses an order message and returns a string representation. This method calls the appropriate parsing method  based on the type of the order.

    - :python:`_parse_kitting_task(self, kitting_task: KittingTask)`: Parses a :python:`KittingTask` object and returns a string representation.

    - :python:`_parse_assembly_task(self, assembly_task: AssemblyTask)`: Parses an :python:`AssemblyTask` object and returns a string representation.

    - :python:`_parse_combined_task(self, combined_task: CombinedTask)`: Parses a :python:`CombinedTask` object and returns a string representation.

    
Executable
----------

.. code-block:: python
    :caption: :file:`tutorial_4.py`
    
    #!/usr/bin/env python3
    '''
    To test this script, run the following commands in separate terminals:
    - ros2 launch ariac_gazebo ariac.launch.py trial_name:=tutorial competitor_pkg:=ariac_tutorials
    - ros2 run ariac_tutorials tutorial_4.py
    '''

    import rclpy
    from ariac_tutorials.competition_interface import CompetitionInterface


    def main(args=None):
        rclpy.init(args=args)
        interface = CompetitionInterface()
        interface.start_competition()
        # The following line enables order displays in the terminal.
        # Set to False to disable.
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

Code Explanation
^^^^^^^^^^^^^^^^

This executable does the following:

    - Initialize the ROS client library.
    - Create an instance of the class :python:`CompetitionInterface` as a ROS node.
    - Start the competition.
    - Log each published message to the terminal.

Build Instructions
------------------

.. code-block:: cmake
    :emphasize-lines: 29
    :caption: :file:`CMakeLists.txt`

    cmake_minimum_required(VERSION 3.8)
    project(ariac_tutorials)

    if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
    endif()

    find_package(ament_cmake REQUIRED)
    find_package(ament_cmake_python REQUIRED)
    find_package(rclcpp REQUIRED)
    find_package(rclpy REQUIRED)
    find_package(ariac_msgs REQUIRED)
    find_package(orocos_kdl REQUIRED)

    # Install the config directory to the package share directory
    install(DIRECTORY 
    config
    DESTINATION share/${PROJECT_NAME}
    )

    # Install Python modules
    ament_python_install_package(${PROJECT_NAME} SCRIPTS_DESTINATION lib/${PROJECT_NAME})

    # Install Python executables
    install(PROGRAMS
    scripts/tutorial_1.py
    scripts/tutorial_2.py
    scripts/tutorial_3.py
    scripts/tutorial_4.py
    DESTINATION lib/${PROJECT_NAME}
    )

    ament_package()


Run the Executable
==================

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
        ros2 launch ariac_gazebo ariac.launch.py trial_name:=tutorial competitor_pkg:=ariac_tutorials



Once the environment is loaded and the competition state is ready, the interface node running in *terminal 1* will start the competition and published orders will be displayed on the standard output.


Outputs
=======


.. code-block:: console
    :caption: terminal 1 output
    :class: no-copybutton
    
    [INFO] [1679433834.697069861] [competition_interface]: Waiting for competition to be ready
    [INFO] [1679433841.299703853] [competition_interface]: Competition state is: idle
    [INFO] [1679433849.094552591] [competition_interface]: Competition state is: ready
    [INFO] [1679433849.095081567] [competition_interface]: Competition is ready. Starting...
    [INFO] [1679433849.100874170] [competition_interface]: Started competition.
    [INFO] [1679433850.165890628] [competition_interface]: 

    ==========================
    Received Order: 2IZJP127
    Priority: False
    Type: Assembly
    ==========================
    AGV(s): [1, 2]
    Station: Assembly Station 1
    Products:
    ==========================
    Part: 🟥 Red Regulator
    Position:
        x: 0.175 (m)
        y: -0.223 (m)
        z: 0.215 (m)
    Orientation:
        roll: 90°
        pitch: 0°
        yaw: -90°
    Install direction:
        x: 0.0
        y: 0.0
        z: -1.0
    Part: 🟥 Red Battery
    Position:
        x: -0.150 (m)
        y: 0.035 (m)
        z: 0.043 (m)
    Orientation:
        roll: 0°
        pitch: 0°
        yaw: 90°
    Install direction:
        x: 0.0
        y: 1.0
        z: 0.0
    Part: 🟥 Red Pump
    Position:
        x: 0.140 (m)
        y: 0.000 (m)
        z: 0.020 (m)
    Orientation:
        roll: 0°
        pitch: 0°
        yaw: -90°
    Install direction:
        x: 0.0
        y: 0.0
        z: -1.0
    Part: 🟥 Red Sensor
    Position:
        x: -0.100 (m)
        y: 0.395 (m)
        z: 0.045 (m)
    Orientation:
        roll: 0°
        pitch: 0°
        yaw: -90°
    Install direction:
        x: 0.0
        y: -1.0
        z: 0.0

    [INFO] [1679433850.166963556] [competition_interface]: 

    ==========================
    Received Order: 2IZJP320
    Priority: False
    Type: Combined
    ==========================
    Station: Assembly Station 3
    Products:
    ==========================
    Part: 🟧 Orange Pump
    Position:
        x: 0.140 (m)
        y: 0.000 (m)
        z: 0.020 (m)
    Orientation:
        roll: 0°
        pitch: 0°
        yaw: -90°
    Install direction:
        x: 0.0
        y: 0.0
        z: -1.0
    Part: 🟧 Orange Sensor
    Position:
        x: -0.100 (m)
        y: 0.395 (m)
        z: 0.045 (m)
    Orientation:
        roll: 0°
        pitch: 0°
        yaw: -90°
    Install direction:
        x: 0.0
        y: -1.0
        z: 0.0

    [INFO] [1679433851.790587939] [competition_interface]: 

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

    [INFO] [1679433861.861995430] [competition_interface]: 

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

    [INFO] [1679433872.665591128] [competition_interface]: 

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

    [INFO] [1679433872.667818858] [competition_interface]: Competition state is: order_announcements_done
