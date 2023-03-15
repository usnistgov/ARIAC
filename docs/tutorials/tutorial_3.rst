
.. _TUTORIAL_3:

=========================================================
Tutorial 3: Reading Data from an Advanced Logical Camera
=========================================================

.. note::
  **Prerequisites**: The code snippets provided in tutorials 1 and 2 will be reused and augmented in this tutorial. The completion of tutorials 1 and 2 is a prerequisite for this tutorial.


In this tutorial you will learn how to:
  - Receive messages from a camera, 
  - Store the data internally as an instance of a class,
  - Display the stored data on the standard output.


Add a Camera to the Environment
--------------------------------

Add a an advanced logical camera to  ``sensors.yaml`` as shown on lines 8-13 in :numref:`sensors-yaml`. 

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

To test  the camera was correctly added to the environment, do the following:

.. code-block:: bash

  cd ~/ariac_ws
  colcon build
  . install/setup.bash
  ros2 launch ariac_gazebo ariac.launch.py trial_name:=tutorial competitor_pkg:=competition_tutorials


You should see the camera above bins 1-4 as shown in the figure below.

.. _fig-advanced-camera-0:
.. figure:: ../images/tutorial3/advanced_camera_0.jpg
   :scale: 70 %
   :align: center
   :figclass: align-center
   :class: with-shadow

Receive Messages from a Camera
---------------------------------

The camera which was added to ``sensors.yaml`` is publishing messages to the topic ``/ariac/sensors/advanced_camera_0/image``. Topics for sensors and cameras are dynamically generated based on the name of the sensors/cameras from ``sensors.yaml`` file. For example, the topic for ``advanced_camera_0`` is ``/ariac/sensors/advanced_camera_0/image``.

Import Modules
^^^^^^^^^^^^^^
Besides the modules imported in tutorial 1, extra modules must be imported in the ``competition_interface.py`` file as seen in :numref:`import-advanced-camera`.

.. code-block:: python
    :caption: Module Imports
    :name: import-advanced-camera
    
    from ariac_msgs.msg import AdvancedLogicalCameraImage as AdvancedLogicalCameraImageMsg
    from ariac_msgs.msg import PartPose as PartPoseMsg
    # For KDL transformations
    import PyKDL
    from geometry_msgs.msg import Pose

Competition Interface Attributes
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

In the class ``CompetitionInterface``, add the following class attributes.

.. code-block:: python
    :caption: Dictionaries for converting PartColor and PartType constants to strings
    :name: class-attributes

    part_colors_ = {
      Part.RED: 'red',
      Part.BLUE: 'blue',
      Part.GREEN: 'green',
      Part.ORANGE: 'orange',
      Part.PURPLE: 'purple',
    }
    '''Dictionary for converting PartColor constants to strings'''    
    
    part_colors_emoji_ = {
      Part.RED: '🟥',
      Part.BLUE: '🟦',
      Part.GREEN: '🟩',
      Part.ORANGE: '🟧',
      Part.PURPLE: '🟪',
    }
    '''Dictionary for displaying an emoji for the part color'''

    part_types_ = {
      Part.BATTERY: 'battery',
      Part.PUMP: 'pump',
      Part.REGULATOR: 'regulator',
      Part.SENSOR: 'sensor',
    }
    '''Dictionary for converting PartType constants to strings'''

Subscriber
^^^^^^^^^^

To read messages published on the topic ``/ariac/sensors/advanced_camera_0/image``, create a subscriber in the ``competition_interface.py`` file as seen in :numref:`competition-interface`.

.. code-block:: python
    :caption: Subscriber to the Camera Topic
    :name: competition-interface
    
    # Subscriber to the logical camera topic
    self.advanced_camera0_sub = self.create_subscription(
        AdvancedLogicalCameraImageMsg,
        '/ariac/sensors/advanced_camera_0/image',
        self.advanced_camera0_cb,
        qos_profile_sensor_data)

    # An instance of the AdvancedLogicalCameraImage class
    self.camera_image_ = None

Camera Callback
^^^^^^^^^^^^^^^


Define the ``AdvancedLogicalCameraImage`` in the ``competition_interface.py`` file as seen in :numref:`advanced-logical-camera-image`. This class is used to store the data from the camera.
Each attribute of this class represents a field of the message type ``AdvancedLogicalCameraImageMsg``.


.. code-block:: python
    :caption: AdvancedLogicalCameraImage Class
    :name: advanced-logical-camera-image
    
    class AdvancedLogicalCameraImage:
      def __init__(self, msg: AdvancedLogicalCameraImageMsg) -> None:
        self.part_poses = msg.part_poses
        self.tray_poses = msg.tray_poses
        self.sensor_pose = msg.sensor_pose


Define the callback for the subscriber as seen in :numref:`advanced-camera-callback`. Each incoming message is converted to an instance of the ``AdvancedLogicalCameraImage`` class and stored in the attribute ``camera_image_``. 

.. code-block:: python
    :caption: Subscriber Callback
    :name: advanced-camera-callback
    
    def advanced_camera0_cb(self, msg: AdvancedLogicalCameraImageMsg):
      self.camera_image_ = AdvancedLogicalCameraImage(msg)




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
        output = '\n\n==========================\n'
        
        sensor_pose: Pose = self.camera_image_.sensor_pose
        
        part_pose: PartPoseMsg
        for part_pose in self.camera_image_.part_poses:
            part_color = CompetitionInterface.part_colors_[part_pose.part.color].capitalize()
            part_color_emoji = CompetitionInterface.part_colors_emoji_[part_pose.part.color]
            part_type = CompetitionInterface.part_types_[part_pose.part.type].capitalize()
            output += f'Part: {part_color_emoji} {part_color} {part_type}\n'
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
        
        # return the resulting pose from frame3
        tf2 = Pose()
        tf2.position.x = frame3.p.x()
        tf2.position.y = frame3.p.y()
        tf2.position.z = frame3.p.z()
        tf2.orientation.x = frame3.M.GetQuaternion()[0]
        tf2.orientation.y = frame3.M.GetQuaternion()[1]
        tf2.orientation.z = frame3.M.GetQuaternion()[2]
        tf2.orientation.w = frame3.M.GetQuaternion()[3]
        
        return tf2



Configure the Executable
--------------------------------

To use this code, create a new file ``read_advanced_camera.py`` in ``competition_tutorials/nodes`` and paste the following code:


.. code-block:: python
    :caption: Display Camera Data
    
    #!/usr/bin/env python3

    import rclpy
    from ariac_tutorials.competition_interface import CompetitionInterface

    def main(args=None):
      rclpy.init(args=args)
      interface = CompetitionInterface()
      interface.start_competition()

      while rclpy.ok():
        try:
          rclpy.spin_once(interface)    
          interface.camera_images_ is not None:
            interface.get_logger().info(interface.parse_advanced_camera_image(interface.camera_image_), throttle_duration_sec=2.0)
        except KeyboardInterrupt:
          break

      interface.destroy_node()
      rclpy.shutdown()


    if __name__ == '__main__':
        main()



This executable creates an instance of the interface, starts the competition and logs each message received from the camera.

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

  cd ~/ariac_ws
  colcon build
  . install/setup.bash
  ros2 run competition_tutorials read_advanced_camera.py


The node will wait until the competition is ready. To start the environment open a second terminal navigate to ``ariac_ws`` and run the following commands:

.. code-block:: bash

  . install/setup.bash
  ros2 launch ariac_gazebo ariac.launch.py trial_name:=tutorial


Once the environment is loaded and the competition state is ready, the interface node running in terminal 1 will start the competition and the sensor will start publishing data.
Each part detected by the camera will be logged to the terminal.