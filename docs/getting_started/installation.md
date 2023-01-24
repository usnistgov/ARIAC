# Installing ARIAC

ARIAC2023 is built for ROS2 Galactic running on Ubuntu 20.04 (Focal)

- Install ROS2 Galactic Desktop using the [instructions on the ROS2 wiki](https://docs.ros.org/en/galactic/Installation/Ubuntu-Install-Debians.html#)

- Create a colcon workspace 

``` bash
source /opt/ros/galactic/setup.bash
mkdir -p ~/ariac_ws/src
cd ~/ariac_ws
```

- Clone this repository into the src directory of the ariac_ws

``` bash
git clone https://github.com/usnistgov/ARIAC.git src/ariac
```

- Install dependencies

``` bash
rosdep install --from-paths src -y --ignore-src
```

- Build the colcon workspace

``` bash
colcon build
```

- Source the workspace 

``` bash
. ~/ariac_ws/install/setup.bash
```

> NOTE: This `setup.bash` file will need to be sourced in all new terminals

## Starting the Environment

If the build process was successful you can launch the ARIAC environment with the defualt trial configuration and user configuration:

``` bash
ros2 launch ariac_gazebo ariac.launch.py
```

To launch the ARIAC environment with a custom configuration

``` bash
ros2 launch ariac_gazebo ariac.launch.py trial_config:={trial.yaml} user_config:={user.yaml}
```

## Moving the robots 

To verify that the robots can be controlled properly you will need three terminals

In terminal 1 start the environment:
``` bash
ros2 launch ariac_gazebo ariac.launch.py
```

In terminal 2 start the moveit node:
``` bash
ros2 launch ariac_moveit_config ariac_robots.launch.py
```

In terminal 3 start the moveit test node:
``` bash
ros2 launch test_competitor moveit_test.launch.py
```

This should start the competition and move each of the robots to the home position. It will also open an RVIZ window showing the robot's planning scene. 


## Running the test competitor

A test competitor has been created to demonstrate how to complete some of the basic functions of working with the ARIAC environment. To run the test competitor you will need three terminals. 

In terminal 1 start the environment using the test_competitor configurations:
``` bash
ros2 launch ariac_gazebo ariac.launch.py trial_config:={test_competitor.yaml} user_config:={test_competitor.yaml}
```

In terminal 2 start the moveit node:
``` bash
ros2 launch ariac_moveit_config ariac_robots.launch.py
```

In terminal 3 start the competitor node:
``` bash
ros2 launch test_competitor competitor.launch.py
```

The test competitor will start the competition, subscribe to the order's topic, and complete orders. 

> NOTE: As of the beta release the test competitor is only capable of completing kitting orders