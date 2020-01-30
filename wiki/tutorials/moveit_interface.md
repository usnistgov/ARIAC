# Overview #

When ARIAC starts, it adds two UR10 arms and with controllers that accept [trajectory_msgs/JointTrajectory](http://docs.ros.org/melodic/api/trajectory_msgs/html/msg/JointTrajectory.html) messages.
The [GEAR interface tutorial](./gear_interface) has already shown you that these messages can be sent manually using the command line.

MoveIt is a tool that can generate these trajectories programmatically.
It feeds the robot's current state and the state of the environment around it to a motion planning algorithm. 
This tutorial will introduce you to controlling the two arms in ARIAC 2019 using [MoveIt](https://moveit.ros.org/).

Teams may modify these configuration files and use them during the competition.
However, this is only one possible approach for controlling the arm.
Teams are free to use alternative motion planning and execution strategies entirely.

**Important, this tutorial and the associated configuration files are published only to provide teams with a starting point for controlling the arm. The ARIAC competition support team will not provide technical support for MoveIt or these configuration files.**


## Setting up your System ##

### Getting MoveIt ###

Run this command to install packages required to complete this tutorial.

```bash
sudo apt-get install ros-melodic-moveit-core \
                       ros-melodic-moveit-kinematics \
                       ros-melodic-moveit-ros-planning \
                       ros-melodic-moveit-ros-move-group \
                       ros-melodic-moveit-planners-ompl \
                       ros-melodic-moveit-ros-visualization \
                       ros-melodic-moveit-simple-controller-manager
```

### Getting the ARIAC Configuration ###

Controlling the arms requires a custom configuration packages.
These will need to be built from source.

Run the following commands to create a catkin workspace, if you have not already done so:

```bash
mkdir -p ~/ariac_ws/src
cd ~/ariac_ws/src
```

And then run this command to download the code

```bash
git clone https://github.com/osrf/universal_robot -b ariac_2019_ur10_moveit_config
```

### Building the ARIAC MoveIt packages ###

Now that you have all the dependencies and code, build the packages.
Source the ROS setup file if you have not already done so.

```bash
source /opt/ros/melodic/setup.bash
```

Now build the workspace with:

```bash
cd ~/ariac_ws
catkin_make install
```

## Using MoveIt ##

Now whenever you want to use MoveIt, make sure to source the workspace the ARIAC MoveIt packages were built in.

```bash
source ~/ariac_ws/install/setup.bash
```

### Running ARIAC and MoveIt ###

Before you can control the arm with MoveIt, you must launch the ARIAC environment.
Run the following command to launch the sample configuration

```bash
roslaunch osrf_gear sample_environment.launch
```

Now run the following command to enable motion planning for arm 1

```bash
# source the workspace if you have not done so already
source ~/ariac_ws/install/setup.bash
roslaunch ur10_moveit_config move_group.launch arm_namespace:=/ariac/arm1
```

And in another terminal run this command to enable motion planning for arm 2

```bash
# source the workspace if you have not done so already
source ~/ariac_ws/install/setup.bash
roslaunch ur10_moveit_config move_group.launch arm_namespace:=/ariac/arm2
```

### Interfacing with MoveIt ###

#### Using RViz ####

RViz is a visualization tool that can represent the state of robots, both simulated and real.
There is an RViz "plugin" for motion planning with MoveIt which can be used to interactively interface with the move_group.
To launch RViz with this plugin set up to control arm 1, run: 

```bash
# source the workspace if you have not done so already
source ~/ariac_ws/install/setup.bash
roslaunch ur10_moveit_config moveit_rviz.launch config:=true arm_namespace:=/ariac/arm1
```

And in another terminal run this to bring up RViz for arm 2:

```bash
# source the workspace if you have not done so already
source ~/ariac_ws/install/setup.bash
roslaunch ur10_moveit_config moveit_rviz.launch config:=true arm_namespace:=/ariac/arm2
```

This should bring up a visualization like the following, which shows the robot state, an "interactive marker" for moving the end effector around, and the goal position for the robot in orange. You may have to [move the RViz camera](wiki.ros.org/rviz/UserGuide#The_different_camera_types) to see the arm.

![ariac_2019_moveit_labeled.png](https://bitbucket.org/repo/pB4bBb/images/14031832-ariac_2019_moveit_labeled.png)

Select the `Planning` tab in the Motion Planning plugin.
Move the end effector using the interactive marker and select `Plan` to get MoveIt to generate a joint trajectory matching that goal position.
If all goes well, you should be able to click `Execute` to send this trajectory to the arm's controller.

You should see that the arm moves in Gazebo, and that this movement is reflected in the visualization in RViz.

#### Programmatically ###

See the official [MoveIt tutorials](https://ros-planning.github.io/moveit_tutorials/) for information about interfacing with moveit programmatically

### Troubleshooting  ###

As previously stated, this tutorial is provided as a starting point.
The ARIAC competition support team will not provide technical support for using MoveIt.