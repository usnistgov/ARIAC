# Tutorial 1: Creating a Competition Package

This tutorial details the steps necessary to create a competition package that is able to interface with the ARIAC competition. This competition package will use a python node to listen to the competition state and call a ROS service to start the competition when ready.

To create the package, navigate to the `src` directory of the `ariac_ws` that was created in [the installation directions](../getting_started/installation.md). Navigate into `ariac_ws/src` and run the package creation command:

```
ros2 pkg create competition_tutorials --build-type ament_cmake
```

Open the newly created folder `/competition_tutorials` in your preferred text editor or IDE

Modify `CMakeLists.txt` to match the following

``` cmake
cmake_minimum_required(VERSION 3.8)
project(competition_tutorials)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(ariac_msgs REQUIRED)

# Install Python modules
ament_python_install_package(${PROJECT_NAME} SCRIPTS_DESTINATION lib/${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  src/start_competition.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
```

Modify `package.xml` to match the following. Make sure to update the maintainer and license tags. 

``` xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>competition_tutorials</name>
  <version>0.0.0</version>
  <description>ROS2 pkg for ARIAC tutorials</description>
  <maintainer email="{your_email}">{your name}</maintainer>
  <license>Apache License 2.0</license>

  <buildtool_depend>ament_cmake</buildtool_depend>

  <depend>rclcpp</depend>
  <depend>rclpy</depend>
  <depend>ariac_msgs</depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

Next, create a python package with the same name as the ros2 package. This python package will include all the python source code for your software. Navigate into `ariac_ws/src/competition_tutorials` and run the following commands:

``` bash
mkdir competition_tutorials
```
``` bash
touch competition_tutorials/__init__.py
```
``` bash
touch competition_tutorials/competition_interface.py
```

Copy the following into `competition_interface.py`: 

``` python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from ariac_msgs.msg import CompetitionState

from std_srvs.srv import Trigger

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
    
    def competition_state_cb(self, msg: CompetitionState):
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
```

This creates a ROS2 python node that is able to call a ROS service to start the competition. To run this function create a python executable inside the `src` directory of this package. Navigate into `ariac_ws/src/competition_tutorials` and run the following commands:

```bash
touch src/start_competition.py
```

Copy the following into `start_competition.py`: 

``` python
#!/usr/bin/env python3

import rclpy
from competition_tutorials.competition_interface import CompetitionInterface

def main(args=None):
    rclpy.init(args=args)

    interface = CompetitionInterface()

    interface.start_competition()

    interface.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

This creates an instance of the `CompetitionInterface` class from `competition_tutorials/competition_interface.py` and calls the `start_competition` method.

Next, you need to build the package and run the node. To do this navigate to `ariac_ws` and run the following commands:

``` bash
colcon build
```
``` bash
. install/setup.bash
```
``` bash
ros2 run competition_tutorials start_competition.py
```

You should this output:
``` 
[INFO] [1677690270.816618425] [start_competition_node]: Waiting for competition to be ready
```

The node will wait until the competition is ready. To start the environment open a second terminal and run the following commands:

``` bash
cd ~/ariac_ws
```
``` bash
. install/setup.bash
```
``` bash
ros2 launch ariac_gazebo ariac.launch.py trial_name:=tutorial_1
```

This should start gazebo. Once the environment is loaded and the competition state is ready, the interface node running in terminal 1 will start the competition. This will activate all sensors, enable the robot controllers, and start the conveyor belt. 


