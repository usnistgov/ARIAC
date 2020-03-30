-------------------------------------------------
- Wiki | [Home](../../README.md) | [Documentation](../documentation/documentation.md) | [Tutorials](../tutorials/tutorials.md) | [Qualifiers](../qualifiers/qualifier.md) | [Finals](../finals/finals.md)
-------------------------------------------------

# Wiki | Tutorials | Installation

Currently, GEAR can only be built from source. We are working on a debian package for GEAR.

## Prerequisites

### Ubuntu Bionic
GEAR requires a machine running [Ubuntu Bionic (18.04)](http://releases.ubuntu.com/18.04/).
If you're running Ubuntu, but not sure which version then run the following command to find out.

```
cat /etc/os-release
```

You should see 18.04 Bionic Beaver in that file.


### Install ROS and Gazebo

- GEAR uses [ROS Melodic](http://www.ros.org/) and [Gazebo 9](http://gazebosim.org/blog/gazebo9) to simulate the environment
  - Follow these instructions to install [ROS Melodic](http://wiki.ros.org/melodic/Installation/Ubuntu).
  - Follow these instructions to install [Gazebo 9](http://gazebosim.org/tutorials?tut=install_ubuntu&cat=install).

**Note**, if you see a crash complaining about missing a symbol in an ignition library, run this to fix it

```bash
sudo apt-get update
sudo apt-get upgrade libignition-math4
```

### Install Catkin Command Line Tools and Create a Catkin Workspace

The [Catkin Command Line Tools (catkin_tools)](https://catkin-tools.readthedocs.io/en/latest/) will be used to build the software.

Follow [these instructions to install catkin_tools](https://catkin-tools.readthedocs.io/en/latest/installing.html).
If you followed the instructions to install ROS above, it should be as simple as

```bash
sudo apt-get update
sudo apt-get install python-catkin-tools
```

### Create a catkin workspace

A [catkin workspace](https://catkin-tools.readthedocs.io/en/latest/quick_start.html#initializing-a-new-workspace) is a place to build ROS packages.
You will need to create one to have a place to build the GEAR packages.
Run these commands to create a workspace `ariac_ws` in your home folder.

```bash
source /opt/ros/melodic/setup.bash
mkdir -p ~/ariac_ws/src
cd ~/ariac_ws
catkin init
```

### Download source code

The source code for `GEAR` is located in the [osrf/gear repository on bitbucket](https://bitbucket.org/osrf/ariac).
GEAR also uses a custom version of [gazebo_ros_pkgs](https://github.com/ros-simulation/gazebo_ros_pkgs/).
Follow these instructions to put the for both of these into the catkin workspace.

```bash
cd ~/ariac_ws/src
git clone https://github.com/usnistgov/ARIAC.git
git clone https://github.com/osrf/ariac-gazebo_ros_pkgs -b ariac-network-melodic
```

### Install GEAR dependencies

The cloned software has its own dependencies that need to be installed.
The tool rosdep can do this automatically.
If you have not used rosdep before, then it must be initialized before running the command above.
Run this if you have not used rosdep before:

```bash
sudo apt-get install python-rosdep
sudo rosdep init
rosdep update
```

Once rosdep has been initialized, use it to install dependencies:

```bash
cd ~/ariac_ws
rosdep install --from-paths ./src --ignore-packages-from-source -y
```

If not installed, make sure you install ros_control:

```bash
sudo apt-get install ros-melodic-ros-control ros-melodic-ros-controllers
```

## Building GEAR

The command `catkin` from the Catkin Command Line Tools is used to build the software.
All of these commands must be run from the root of the workspace.
Use `cd` to get there in a terminal.

```bash
cd ~/ariac_ws
```

For more information see the [documentation for `catkin build`](https://catkin-tools.readthedocs.io/en/latest/verbs/catkin_build.html).


### Building all packages in the workspace
Run this to build all of the packages in the workspace.

```bash
catkin build
```

Once all packages have been built once, you can rebuild them individually as needed.

### Building a single package in the workspace
The command to build a single package by name is:

```
catkin build --no-deps PACKAGE_NAME
```

For example, run this to build just the main GEAR package:

```bash
catkin build --no-deps nist_gear
```

### Building Release versus Debug builds

To build packages in debug mode with debug symbols, add `cmake` args specifying the build type.

For example, to rebuild all package in debug mode use:

```
catkin build --cmake-args -DCMAKE_BUILD_TYPE=Debug
```

To build in release use:

```
catkin build --cmake-args -DCMAKE_BUILD_TYPE=Release
```

For more info see the [documentation on CMAKE_BUILD_TYPE](https://cmake.org/cmake/help/v3.5/variable/CMAKE_BUILD_TYPE.html).
## Testing GEAR

Run this command to run both unit and integration tests to make sure GEAR is working correctly.

```
catkin test --no-deps nist_gear test_ariac
```

The above command says to run tests for the `nist_gear` and `test_ariac` package.
Unit tests (shorter, but less coverage) are in `nist_gear`.
Long running integration tests are in `test_ariac`.

## Running gear

After building all packages, the workspace must be sourced before building.
Always source the workspace in a new terminal.
Never source the workspace in a terminal where you run `catkin build`.


To source the workspace:

```
source ~/ariac_ws/devel/setup.bash
```

Make sure it works by running the ARIAC sample environment.

```
roslaunch nist_gear sample_environment.launch
```

Note the very first launch may take a while because Gazebo downloads models from the model database the first time it runs.

-------------------------------------------------
- Wiki | [Home](../../README.md) | [Documentation](../documentation/documentation.md) | [Tutorials](../tutorials/tutorials.md) | [Qualifiers](../qualifiers/qualifier.md) | [Finals](../finals/finals.md)

-------------------------------------------------
