
.. _TUTORIAL_3:

=========================================================
Tutorial 3: Reading Data from an Advanced Logical Camera
=========================================================

.. note::
  **Prerequisites**: The code snippets provided in tutorials 1 and 2 will be reused and augmented in this tutorial. The completion of tutorials 1 and 2 is a prerequisite for this tutorial.


This tutorial covers the following steps:
  - Receive messages from a camera, 
  - Store the data internally as an instance of a class,
  - Display the stored data on the standard output.


Add a Camera to the Environment
--------------------------------

Add an advanced logical camera to  ``sensors.yaml`` (lines 8-13 in :numref:`sensors-yaml`). 

.. code-block:: yaml
    :caption: sensors.yaml
    :name: sensors-yaml
    :emphasize-lines: 8, 9, 10, 11, 12, 13
    :linenos:
    
    sensors:
      breakbeam_0:
        type: break_beam
        visualize_fov: true
        pose:
          xyz: [-0.36, 3.5, 0.88]
          rpy: [0, 0, pi]
      advanced_camera_0:
        type: advanced_logical_camera
        visualize_fov: true
        pose:
          xyz: [-2.286, 2.96, 1.8]
          rpy: [pi, pi/2, 0]




Test the Camera
^^^^^^^^^^^^^^^^^^

To test  the camera was correctly added to the environment:

.. code-block:: bash

  cd ~/ariac_ws
  colcon build
  . install/setup.bash
  ros2 launch ariac_gazebo ariac.launch.py trial_name:=tutorial competitor_pkg:=competition_tutorials


You should see the camera above bins 1-4 as shown in the figure below.

.. _fig-advanced-camera-0:
.. figure:: ../images/tutorial3/advanced_camera_0.jpg
   :align: center

.. Receive Messages from a Camera
.. ---------------------------------

.. The camera which was added to ``sensors.yaml`` is publishing messages to the topic ``/ariac/sensors/advanced_camera_0/image``. Topics for sensors and cameras are dynamically generated based on the name of the sensors/cameras from ``sensors.yaml`` file. For example, the topic for ``advanced_camera_0`` is ``/ariac/sensors/advanced_camera_0/image``.

Import Modules
^^^^^^^^^^^^^^
The modules shown in  :numref:`import-advanced-camera` must be imported in ``competition_interface.py``.

.. code-block:: python
    :caption: Module Imports
    :name: import-advanced-camera
    
    import rclpy
    import PyKDL
    from dataclasses import dataclass
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.parameter import Parameter
    from geometry_msgs.msg import Pose

    from ariac_msgs.msg import (
        CompetitionState,
        Part,
        AdvancedLogicalCameraImage as AdvancedLogicalCameraImageMsg,
        PartPose as PartPoseMsg,
        KitTrayPose as KitTrayPoseMsg,
    )

    from std_srvs.srv import Trigger

Competition Interface
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The class ``CompetitionInterface`` used in this tutorial is shown in :numref:`competitioninterface`. The content of this class is described in the following sections.

.. code-block:: python
    :caption: CompetitionInterface class
    :name: competitioninterface

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
            # Subscriber to the logical camera topic
            self._advanced_camera0_sub = self.create_subscription(
                AdvancedLogicalCameraImageMsg,
                '/ariac/sensors/advanced_camera_0/image',
                self.advanced_camera0_cb,
                qos_profile_sensor_data)
            # Store each camera image as an AdvancedLogicalCameraImage object
            self._camera_image: AdvancedLogicalCameraImage = None

        @property
        def camera_image(self):
            '''Property for the camera images.'''
            return self._camera_image

        def competition_state_cb(self, msg: CompetitionStateMsg):
            '''Callback for the topic /ariac/competition_state

            Arguments:
                msg -- CompetitionState message
            '''
            # Log if competition state has changed
            if self._competition_state != msg.competition_state:
                self.get_logger().info(
                    f'Competition state is: \
                    {CompetitionInterface._competition_states[msg.competition_state]}',
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

        def advanced_camera0_cb(self, msg: AdvancedLogicalCameraImageMsg):
            '''Callback for the topic /ariac/sensors/advanced_camera_0/image

            Arguments:
                msg -- AdvancedLogicalCameraImage message
            '''
            self._camera_image = AdvancedLogicalCameraImage(msg.part_poses,
                                                            msg.tray_poses,
                                                            msg.sensor_pose)

        def multiply_pose(self, pose1: Pose, pose2: Pose):
            '''
            Use KDL to multiply two poses together.

            Args:
                pose1 (Pose): Pose of the first frame
                pose2 (Pose): Pose of the second frame

            Returns:
                Pose: Pose of the resulting frame
            '''

            frame1 = PyKDL.Frame(PyKDL.Rotation.Quaternion(pose1.orientation.x,
                                                        pose1.orientation.y,
                                                        pose1.orientation.z,
                                                        pose1.orientation.w),
                                PyKDL.Vector(pose1.position.x, pose1.position.y, pose1.position.z))

            frame2 = PyKDL.Frame(PyKDL.Rotation.Quaternion(pose2.orientation.x,
                                                        pose2.orientation.y,
                                                        pose2.orientation.z,
                                                        pose2.orientation.w),
                                PyKDL.Vector(pose2.position.x, pose2.position.y, pose2.position.z))

            frame3: PyKDL.Frame = frame1 * frame2

            tf2 = Pose()
            tf2.position.x = frame3.p.x()
            tf2.position.y = frame3.p.y()
            tf2.position.z = frame3.p.z()
            tf2.orientation.x = frame3.M.GetQuaternion()[0]
            tf2.orientation.y = frame3.M.GetQuaternion()[1]
            tf2.orientation.z = frame3.M.GetQuaternion()[2]
            tf2.orientation.w = frame3.M.GetQuaternion()[3]

            # return the resulting pose from frame3
            return tf2

        def parse_advanced_camera_image(self):
            '''
            Parse an AdvancedLogicalCameraImage message and return a string representation.


            Args:
                image (AdvancedLogicalCameraImage): Object of type AdvancedLogicalCameraImage
            '''
            output = '\n\n==========================\n'

            sensor_pose: Pose = self._camera_image._sensor_pose

            part_pose: PartPoseMsg
            
            counter = 1
            for part_pose in self._camera_image._part_poses:
                part_color = CompetitionInterface._part_colors[part_pose.part.color].capitalize()
                part_color_emoji = CompetitionInterface._part_colors_emoji[part_pose.part.color]
                part_type = CompetitionInterface._part_types[part_pose.part.type].capitalize()
                output += f'Part {counter}: {part_color_emoji} {part_color} {part_type}\n'
                output += '==========================\n'
                output += 'Camera Frame\n'
                output += '==========================\n'
                position = f'x: {part_pose.pose.position.x}\n\t\ty: {part_pose.pose.position.y}\n\t\tz: {part_pose.pose.position.z}'
                orientation = f'x: {part_pose.pose.orientation.x}\n\t\ty: {part_pose.pose.orientation.y}\n\t\tz: {part_pose.pose.orientation.z}\n\t\tw: {part_pose.pose.orientation.w}'

                output += '\tPosition:\n'
                output += f'\t\t{position}\n'
                output += '\tOrientation:\n'
                output += f'\t\t{orientation}\n'
                output += '==========================\n'
                output += 'World Frame\n'
                output += '==========================\n'
                part_world_pose = self.multiply_pose(sensor_pose, part_pose.pose)
                position = f'x: {part_world_pose.position.x}\n\t\ty: {part_world_pose.position.y}\n\t\tz: {part_world_pose.position.z}'
                orientation = f'x: {part_world_pose.orientation.x}\n\t\ty: {part_world_pose.orientation.y}\n\t\tz: {part_world_pose.orientation.z}\n\t\tw: {part_world_pose.orientation.w}'

                output += '\tPosition:\n'
                output += f'\t\t{position}\n'
                output += '\tOrientation:\n'
                output += f'\t\t{orientation}\n'
                output += '==========================\n'
                
                counter += 1

            return output


Class Attributes
^^^^^^^^^^^^^^^^

The class attributes ``_part_colors`` and ``_part_types`` are dictionaries that map the integer values of the part color and type to their string representations. The class attribute ``_part_colors_emoji`` is a dictionary that maps the integer values of the part color to their emoji representations.
These dictionaries are mainly used to display the part color and type in a human-readable format.

Subscriber
^^^^^^^^^^

A subscriber to the topic ``/ariac/sensors/advanced_camera_0/image`` is shown in :numref:`tutorial3-subscriber`. Topics for sensors and cameras are dynamically generated based on the name of the sensors/cameras from ``sensors.yaml`` file. For example, the topic for ``advanced_camera_0`` is ``/ariac/sensors/advanced_camera_0/image``.
Each message received on this topic is stored in the attribute ``_camera_image``. This attribute is an instance of the class ``AdvancedLogicalCameraImage``, which is defined in :numref:`AdvancedLogicalCameraImage`.

.. code-block:: python
    :caption: Subscriber to the Camera Topic
    :name: tutorial3-subscriber
    
    # Subscriber to the logical camera topic
    self._advanced_camera0_sub = self.create_subscription(
        AdvancedLogicalCameraImageMsg,
        '/ariac/sensors/advanced_camera_0/image',
        self.advanced_camera0_cb,
        qos_profile_sensor_data)

    # Store each camera image as an AdvancedLogicalCameraImage object
    self._camera_image: AdvancedLogicalCameraImage = None


.. code-block:: python
    :caption: AdvancedLogicalCameraImage class
    :name: AdvancedLogicalCameraImage
    
    @dataclass
    class AdvancedLogicalCameraImage:
    '''
    Class to store information about a AdvancedLogicalCameraImageMsg.
    '''
        _part_poses: PartPoseMsg
        _tray_poses: KitTrayPoseMsg
        _sensor_pose: Pose

Camera Callback
^^^^^^^^^^^^^^^

The callback for the camera subscriber is seen in :numref:`advanced-camera-callback`. 
Each incoming message is converted to an instance of the ``AdvancedLogicalCameraImage`` class and stored in the attribute ``camera_image_``. 

.. code-block:: python
    :caption: Subscriber Callback
    :name: advanced-camera-callback
    
    def advanced_camera0_cb(self, msg: AdvancedLogicalCameraImageMsg):
        '''Callback for the topic /ariac/sensors/advanced_camera_0/image

        Arguments:
            msg -- AdvancedLogicalCameraImage message
        '''
        self._camera_image = AdvancedLogicalCameraImage(msg.part_poses,
                                                        msg.tray_poses,
                                                        msg.sensor_pose)


Parse Stored Camera Image
--------------------------------


To parse the attribute ``camera_image_`` (refer to :numref:`competition-interface`), create a new method in the ``competition_interface.py`` file as seen in :numref:`parse-advanced-camera-image`.
This method parses the attribute ``camera_image_``  and prints its content to the standard output. The output is printed in the following format:

  - Emoji for the part color using the class attribute ``part_colors_emoji_``.
  - Part color using the class attribute ``part_colors_``.
  - Part type using the class attribute ``part_types_``.
  - Part pose in the camera frame: This is the pose returned by the camera.
  - Part pose in the world frame: This is calculated by multiplying the camera pose with the part pose in the camera frame. This multiplication is done using the method ``multiply_pose`` (see  :numref:`multiply-pose`).

.. code-block:: python
    :caption: Parse AdvancedLogicalCameraImage Instance
    :name: parse-advanced-camera-image
    
    def parse_advanced_camera_image(self):
            '''
            Parse an AdvancedLogicalCameraImage message and return a string representation.
            '''
            output = '\n\n==========================\n'

            sensor_pose: Pose = self._camera_image._sensor_pose

            part_pose: PartPoseMsg
            
            counter = 1
            for part_pose in self._camera_image._part_poses:
                part_color = CompetitionInterface._part_colors[part_pose.part.color].capitalize()
                part_color_emoji = CompetitionInterface._part_colors_emoji[part_pose.part.color]
                part_type = CompetitionInterface._part_types[part_pose.part.type].capitalize()
                output += f'Part {counter}: {part_color_emoji} {part_color} {part_type}\n'
                output += '==========================\n'
                output += 'Camera Frame\n'
                output += '==========================\n'
                position = f'x: {part_pose.pose.position.x}\n\t\ty: {part_pose.pose.position.y}\n\t\tz: {part_pose.pose.position.z}'
                orientation = f'x: {part_pose.pose.orientation.x}\n\t\ty: {part_pose.pose.orientation.y}\n\t\tz: {part_pose.pose.orientation.z}\n\t\tw: {part_pose.pose.orientation.w}'

                output += '\tPosition:\n'
                output += f'\t\t{position}\n'
                output += '\tOrientation:\n'
                output += f'\t\t{orientation}\n'
                output += '==========================\n'
                output += 'World Frame\n'
                output += '==========================\n'
                part_world_pose = self.multiply_pose(sensor_pose, part_pose.pose)
                position = f'x: {part_world_pose.position.x}\n\t\ty: {part_world_pose.position.y}\n\t\tz: {part_world_pose.position.z}'
                orientation = f'x: {part_world_pose.orientation.x}\n\t\ty: {part_world_pose.orientation.y}\n\t\tz: {part_world_pose.orientation.z}\n\t\tw: {part_world_pose.orientation.w}'

                output += '\tPosition:\n'
                output += f'\t\t{position}\n'
                output += '\tOrientation:\n'
                output += f'\t\t{orientation}\n'
                output += '==========================\n'
                
                counter += 1

            return output

.. code-block:: python
    :caption: Transform using KDL frames
    :name: multiply-pose
    
    def multiply_pose(self, pose1: Pose, pose2: Pose):
        '''
        Use KDL to multiply two poses together.

        Args:
            pose1 (Pose): Pose of the first frame
            pose2 (Pose): Pose of the second frame

        Returns:
            Pose: Pose of the resulting frame
        '''

        frame1 = PyKDL.Frame(PyKDL.Rotation.Quaternion(pose1.orientation.x,
                                                       pose1.orientation.y,
                                                       pose1.orientation.z,
                                                       pose1.orientation.w),
                             PyKDL.Vector(pose1.position.x, pose1.position.y, pose1.position.z))

        frame2 = PyKDL.Frame(PyKDL.Rotation.Quaternion(pose2.orientation.x,
                                                       pose2.orientation.y,
                                                       pose2.orientation.z,
                                                       pose2.orientation.w),
                             PyKDL.Vector(pose2.position.x, pose2.position.y, pose2.position.z))

        frame3: PyKDL.Frame = frame1 * frame2

        tf2 = Pose()
        tf2.position.x = frame3.p.x()
        tf2.position.y = frame3.p.y()
        tf2.position.z = frame3.p.z()
        tf2.orientation.x = frame3.M.GetQuaternion()[0]
        tf2.orientation.y = frame3.M.GetQuaternion()[1]
        tf2.orientation.z = frame3.M.GetQuaternion()[2]
        tf2.orientation.w = frame3.M.GetQuaternion()[3]

        # return the resulting pose from frame3
        return tf2



Configure the Executable
--------------------------------

To test this tutorial, create a new file ``read_advanced_camera.py`` in ``competition_tutorials/nodes`` and paste the following code:


.. code-block:: python
    :caption: Display Camera Data
    
    #!/usr/bin/env python3

    import rclpy
    from ariac_tutorials.tutorial3 import CompetitionInterface


    def main(args=None):
        rclpy.init(args=args)
        interface = CompetitionInterface()
        interface.start_competition()

        while rclpy.ok():
        try:
        rclpy.spin_once(interface)
        # interface.get_logger().info(
            # f'Number of parts detected: {len(interface.camera_images)}', throttle_duration_sec=2.0)

        if interface.camera_image is not None:
            interface.get_logger().info(interface.parse_advanced_camera_image(), throttle_duration_sec=2.0)
        except KeyboardInterrupt:
        break

        interface.destroy_node()
        rclpy.shutdown()


    if __name__ == '__main__':
    main()



This executable creates an instance of the interface, starts the competition and logs the content of ``_camera_image`` every 2 seconds.

Update CMakelists.txt
^^^^^^^^^^^^^^^^^^^^^^

Update ``CMakeLists.txt`` to add ``read_advanced_camera.py`` as an executable.

.. code-block:: cmake

  # Install Python executables
  install(PROGRAMS
    src/start_competition.py
    src/read_advanced_camera.py
    DESTINATION lib/${PROJECT_NAME}
  )


Run the Executable
--------------------------------

Next, build the package and run the node. To do this navigate to ``ariac_ws`` and run the following commands:


.. code-block:: bash
    :caption: Terminal 1: Run the node for tutorial 3

  cd ~/ariac_ws
  colcon build
  . install/setup.bash
  ros2 run competition_tutorials read_advanced_camera.py


The node will wait until the competition is ready. To start the environment open a second terminal navigate to ``ariac_ws`` and run the following commands:

.. code-block:: bash
    :caption: Terminal 2: Start the environment

  . install/setup.bash
  ros2 launch ariac_gazebo ariac.launch.py trial_name:=tutorial


Once the environment is loaded and the competition state is ready, the interface node running in Terminal 1 will start the competition and the sensor will start publishing data.
Each part detected by the camera will be logged to the terminal.