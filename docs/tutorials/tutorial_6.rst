
.. _TUTORIAL_6:

============================================
Tutorial 6: Changing the State of a Gripper
============================================

.. note::
  **Prerequisites**: Tutorial 1 should be completed before starting this tutorial.

This tutorial shows how to use service calls to enable and disable the gripper of a robot. The gripper can be enabled and disabled using the ``/ariac/gripper/control`` service. The service takes a boolean as input. If the input is ``True``, the gripper is enabled. If the input is ``False``, the gripper is disabled. The service returns a boolean as output. If the output is ``True``, the gripper was successfully enabled or disabled. If the output is ``False``, the gripper was not successfully enabled or disabled.

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
        ├── move_agvs.py                (from tutorial 5)
        └── change_gripper_state.py     (new)





Competition Interface
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The competition interface used in this tutorial is shown in :numref:`competitioninterface-tutorial6`.

.. code-block:: python
    :caption: Competition interface for tutorial 6
    :name: competitioninterface-tutorial6
    :emphasize-lines: 6,7,11
    :linenos:

    #!/usr/bin/env python3

    import rclpy
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.time import Duration
    from rclpy.qos import qos_profile_sensor_data

    from ariac_msgs.msg import (
        CompetitionState as CompetitionStateMsg,
        VacuumGripperState,
    )

    from ariac_msgs.srv import VacuumGripperControl
    from std_srvs.srv import Trigger


    class CompetitionInterface(Node):
        '''
        Class for a competition interface node.

        Args:
            Node (rclpy.node.Node): Parent class for ROS nodes

        Raises:
            KeyboardInterrupt: Exception raised when the user uses Ctrl+C to kill a process
        '''

        gripper_states_ = {
            True: 'enabled',
            False: 'disabled'
        }
        '''Dictionary for converting VacuumGripperState constants to strings'''

        _competition_states = {
            CompetitionStateMsg.IDLE: 'idle',
            CompetitionStateMsg.READY: 'ready',
            CompetitionStateMsg.STARTED: 'started',
            CompetitionStateMsg.ORDER_ANNOUNCEMENTS_DONE: 'order_announcements_done',
            CompetitionStateMsg.ENDED: 'ended',
        }
        '''Dictionary for converting CompetitionState constants to strings'''

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

            # Subscriber to the floor gripper state topic
            self._floor_robot_gripper_state_sub = self.create_subscription(
                VacuumGripperState,
                '/ariac/floor_robot_gripper_state',
                self.floor_robot_gripper_state_cb,
                qos_profile_sensor_data)

            # Service client for turning on/off the vacuum gripper on the floor robot
            self._floor_gripper_enable = self.create_client(
                VacuumGripperControl,
                "/ariac/floor_robot_enable_gripper")

            # Attribute to store the current state of the floor robot gripper
            self._floor_robot_gripper_state = VacuumGripperState()

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

        def floor_robot_gripper_state_cb(self, msg: VacuumGripperState):
            '''Callback for the topic /ariac/floor_robot_gripper_state

            Arguments:
                msg -- VacuumGripperState message
            '''
            self._floor_robot_gripper_state = msg

        def set_floor_robot_gripper_state(self, state):
            '''Set the gripper state of the floor robot.

            Arguments:
                state -- True to enable, False to disable

            Raises:
                KeyboardInterrupt: Exception raised when the user presses Ctrl+C
            '''
            if self._floor_robot_gripper_state.enabled == state:
                self.get_logger().warn(f'Gripper is already {self.gripper_states_[state]}')
                return

            request = VacuumGripperControl.Request()
            request.enable = state

            future = self._floor_gripper_enable.call_async(request)

            try:
                rclpy.spin_until_future_complete(self, future)
            except KeyboardInterrupt as kb_error:
                raise KeyboardInterrupt from kb_error

            if future.result().success:
                self.get_logger().info(f'Changed gripper state to {self.gripper_states_[state]}')
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

           
