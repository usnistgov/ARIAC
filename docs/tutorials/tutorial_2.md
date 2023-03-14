# Tutorial 2: Reading Data from a Sensor

Please complete the steps in [Tutorial 1](./tutorial_1.md) before this tutorial.

This tutorial will add functionality to the competition interface to read data from a sensor and output that data to the terminal.

To start, create a [custom sensor configuration](../competition/sensors.md). Navigate to `ariac_ws/src/competition_tutorials` and run the following command:

``` bash
mkdir config
```

``` bash
touch config/sensors.yaml
```

Add the following to `sensors.yaml`:

``` yaml
sensors:
  breakbeam_0:
    type: break_beam
    visualize_fov: true
    pose:
      xyz: [-0.36, 3.5, 0.88]
      rpy: [0, 0, pi]
```
This will add a break beam sensor named `breakbeam_0` to a spot near the start of the conveyor belt. 

To allow for the competition software to be able to find the sensor configuration it must be added to the share directory of the package. Below the existing 
`install(PROGRAMS
  src/start_competition.py
  DESTINATION lib/${PROJECT_NAME}
)`,
add the lines:

``` cmake
# Install the config directory to the package share directory
install(DIRECTORY 
  config
  DESTINATION share/${PROJECT_NAME}
)
```

To test that this worked, build the workspace. In `ariac_ws` run

```bash
colcon build
```

```bash
. install/setup.bash
```

Then start the environment with:

```
ros2 launch ariac_gazebo ariac.launch.py trial_name:=tutorial competitor_pkg:=competition_tutorials 
```

You should see a break beam sensor on the right side of the conveyor belt.

![part breaking sensor beam](../images/tutorial_2_image1.png)

To read the data from the sensor update `competition_interface.py` to match:

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from ariac_msgs.msg import CompetitionState
from std_srvs.srv import Trigger

from rclpy import qos
from ariac_msgs.msg import BreakBeamStatus

class CompetitionInterface(Node):
    # Dictionary to convert competition_state constants to strings
    states = {
        CompetitionState.IDLE: 'idle',
        CompetitionState.READY: 'ready',
        CompetitionState.STARTED: 'started',
        CompetitionState.ORDER_ANNOUNCEMENTS_DONE: 'order_announcements_done',
        CompetitionState.ENDED: 'ended',
    }
    
    def __init__(self):
        super().__init__('start_competition_node')

        self.competition_state = None

        self.subscription = self.create_subscription(
            CompetitionState, 
            '/ariac/competition_state',
            self.competition_state_cb,
            10)
        
        self.starter = self.create_client(Trigger, '/ariac/start_competition')

        # Create subscription for breakbeam_0
        self.part_count = 0
        self.object_detected = False
        self.break_beam_sub = self.create_subscription(
            BreakBeamStatus, 
            '/ariac/sensors/breakbeam_0/status',
            self.breakbeam0_cb,
            qos.qos_profile_sensor_data)
    
    def competition_state_cb(self, msg: CompetitionState):
        # Log if competition state has changed
        if self.competition_state != msg.competition_state:
            self.get_logger().info(
                f'Competition state is: {self.states[msg.competition_state]}',
                throttle_duration_sec=1.0)
        self.competition_state = msg.competition_state

    def start_competition(self):
        self.get_logger().info('Waiting for competition to be ready')

        # Wait for competition to be ready
        while (self.competition_state != CompetitionState.READY):
            try:
                rclpy.spin_once(self)
            except KeyboardInterrupt:
                return
        
        self.get_logger().info('Competition is ready. Starting...')

        # Call ROS service to start competition
        while not self.starter.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /ariac/start_competition to be available...')

        # Create trigger request and call starter service
        request = Trigger.Request()
        future = self.starter.call_async(request)

        # Wait until the service call is completed
        rclpy.spin_until_future_complete(self, future)

        if future.result().success:
            self.get_logger().info('Started competition.')
        else:
            self.get_logger().info('Unable to start competition')

    def breakbeam0_cb(self, msg: BreakBeamStatus):
        if not self.object_detected and msg.object_detected:
            self.part_count += 1

        self.object_detected = msg.object_detected

```

Three major things are added to `competition_interface.py`. The first is to the imports:

``` python
from rclpy import qos
from ariac_msgs.msg import BreakBeamStatus
```

To create a subscription to the breakbeam sensor, the subscriber needs to match the QOS for the for the sensor. This is shown when the subscriber is created. The ROS msg `BreakBeamStatus` which is the msg type that the sensor publishes is also added to the imports.

The next change is in the `__init__` method of the interface

``` python
# Create subscription for breakbeam_0
self.part_count = 0
self.object_detected = False
self.break_beam_sub = self.create_subscription(
    BreakBeamStatus, 
    '/ariac/sensors/breakbeam_0/status',
    self.breakbeam0_cb,
    qos.qos_profile_sensor_data)
```

Two attributes are added to the interface (`part_count` and `objected_detected`). These are used to calculate the total part count on the conveyor belt.

The subscriber is also created. The arguments for creating a subscriber are: `msg_type`, `topic` `callback_function` and `qos_profile`. The msg type is `BreakBeamStatus`, the topic name is `/ariac/sensors/breakbeam_0/status`, the callback is `self.breakbeam0_cb` and the qos profile is `qos_profile_sensor_data`. The callback is a function inside of the interface that will entered every time a message is received on that topic.

The final change is the callback method:

``` python
def breakbeam0_cb(self, msg: BreakBeamStatus):
    if not self.object_detected and msg.object_detected:
        self.part_count += 1

    self.object_detected = msg.object_detected
```

This function receives a `BreakBeamStatus` msg every time the sensor publishes data. It increments the `part_count` attribute every time a part breaks the beam for the first time. 

To use this code create a new file `read_sensor.py` in `competition_tutorials/src` and paste the following code:

``` python
#!/usr/bin/env python3

import rclpy
from competition_tutorials.competition_interface import CompetitionInterface

def main(args=None):
    rclpy.init(args=args)

    interface = CompetitionInterface()

    interface.start_competition()

    while rclpy.ok():
        try:
            rclpy.spin_once(interface)
            interface.get_logger().info(f'Part Count: {interface.part_count}', throttle_duration_sec=2.0)
        except KeyboardInterrupt:
            break

    interface.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

```

This executable creates an instance of the interface, starts the competition and logs the `part_count` at a two second interval. 

Update `CMakeLists.txt` to add `read_sensor.py` as an executable

``` cmake
# Install Python executables
install(PROGRAMS
  src/start_competition.py
  src/read_sensor.py
  DESTINATION lib/${PROJECT_NAME}
)
```

Next, you need to build the package and run the node. To do this navigate to `ariac_ws` and run the following commands:

``` bash
colcon build
```
``` bash
. install/setup.bash
```
``` bash
ros2 run competition_tutorials read_sensor.py
```

The node will wait until the competition is ready. To start the environment open a second terminal navigate to `ariac_ws` and run the following commands:

``` bash
. install/setup.bash
```
``` bash
ros2 launch ariac_gazebo ariac.launch.py trial_name:=tutorial
```

Once the environment is loaded and the competition state is ready, the interface node running in terminal 1 will start the competition and the sensor will start publishing data. You should see the the part count output increasing as part on the conveyor break the sensor beam. 

![part breaking sensor beam](../images/tutorial_2_image2.png)
