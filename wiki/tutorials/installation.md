Wiki | [Home](../../README.md) | [Documentation](../documentation/documentation.md) | [Tutorials](../tutorials/tutorials.md) | [Qualifiers](../qualifiers/qualifier.md) | [Finals](../finals/finals.md)

-------------------------------------------------

- [Wiki | Tutorials | Installation](#wiki--tutorials--installation)
  - [Prerequisites](#prerequisites)
    - [Ubuntu Bionic](#ubuntu-bionic)
    - [Install ROS and Gazebo](#install-ros-and-gazebo)
    - [Install Catkin Command Line Tools and Create a Catkin Workspace](#install-catkin-command-line-tools-and-create-a-catkin-workspace)
    - [Create a Catkin Workspace](#create-a-catkin-workspace)
    - [Download Source Code](#download-source-code)
    - [Install GEAR Dependencies](#install-gear-dependencies)
  - [Build GEAR](#build-gear)
    - [Build all packages in the workspace](#build-all-packages-in-the-workspace)
    - [Build a single package in the workspace](#build-a-single-package-in-the-workspace)
    - [Build Release versus Debug builds](#build-release-versus-debug-builds)
  - [Run GEAR](#run-gear)

# Wiki | Tutorials | Installation

Currently, GEAR can only be built from source.

## Prerequisites

### Ubuntu Bionic

GEAR requires a machine running [Ubuntu Bionic (18.04)](http://releases.ubuntu.com/18.04/).
If you're running Ubuntu, but not sure which version then run the following command to find out.

```bash
$ cat /etc/os-release
```

You should see 18.04 Bionic Beaver in that file.

### Install ROS and Gazebo

- GEAR uses [ROS Melodic](http://www.ros.org/) and at least Gazebo 9.16.0 to simulate the environment.
  - Follow these instructions to install [ROS Melodic](http://wiki.ros.org/melodic/Installation/Ubuntu).
  
  - The default version of Gazebo (9.0) that comes with ROS Melodic will not work with the current version of GEAR. Gazebo 9.16.0 is recommended and can be installed with the following instructions.
  
  ```bash
  $ sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
  $ wget https://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
  $ sudo apt update
  $ sudo apt upgrade
  ```

**Note**, if you see a crash complaining about missing a symbol in an ignition library, run this to fix it

```bash
$ sudo apt-get update
$ sudo apt-get upgrade libignition-math4
```

### Install Catkin Command Line Tools and Create a Catkin Workspace

The [Catkin Command Line Tools (catkin_tools)](https://catkin-tools.readthedocs.io/en/latest/) will be used to build the software.

Follow [these instructions to install catkin_tools](https://catkin-tools.readthedocs.io/en/latest/installing.html).
If you followed the instructions to install ROS above, it should be as simple as

```bash
$ sudo apt-get update
$ sudo apt-get install python-catkin-tools
```

### Create a Catkin Workspace

A [catkin workspace](https://catkin-tools.readthedocs.io/en/latest/quick_start.html#initializing-a-new-workspace) is a place to build ROS packages.
You will need to create one to have a place to build the GEAR packages.
Run these commands to create a workspace `ariac_ws` in your home folder.

```bash
$ source /opt/ros/melodic/setup.bash
$ mkdir -p ~/ariac_ws/src
$ cd ~/ariac_ws
$ catkin init
```

### Download Source Code

The source code for `GEAR` is located in the [nist_gear repository on github](https://github.com/usnistgov/ARIAC.git). GEAR also uses a custom version of [gazebo_ros_pkgs](https://github.com/ros-simulation/gazebo_ros_pkgs/).
Follow these instructions to put the for both of these into the catkin workspace.

```bash
$ cd ~/ariac_ws/src
$ git clone https://github.com/usnistgov/ARIAC.git
$ git clone https://github.com/osrf/ariac-gazebo_ros_pkgs -b ariac-network-melodic
```

### Install GEAR Dependencies

The cloned software has its own dependencies that need to be installed. The tool rosdep can do this automatically. If you have not used rosdep before, then it must be initialized before running the command above. Run this if you have not used rosdep before:

```bash
$ sudo apt-get install python-rosdep
$ sudo rosdep init
$ rosdep update
```

Once rosdep has been initialized, use it to install dependencies:

```bash
$ cd ~/ariac_ws
$ rosdep install --from-paths ./src --ignore-packages-from-source -y
```

If not installed, make sure you install ros_control:

```bash
$ sudo apt install "ros-melodic-ros-control*" "ros-melodic-control*" "ros-melodic-gazebo-ros-control*"
```

## Build GEAR

The command `catkin` from the Catkin Command Line Tools is used to build the software.
All of these commands must be run from the root of the workspace.
Use `cd` to get there in a terminal.

```bash
$ cd ~/ariac_ws
```

For more information see the [documentation for `catkin build`](https://catkin-tools.readthedocs.io/en/latest/verbs/catkin_build.html).

### Build all packages in the workspace

Run this to build all of the packages in the workspace.

```bash
$ catkin build
```

Once all packages have been built once, you can rebuild them individually as needed.

### Build a single package in the workspace

The command to build a single package by name is:

```bash
$ catkin build --no-deps PACKAGE_NAME
```

For example, run this to build just the main GEAR package:

```bash
$ catkin build --no-deps nist_gear
```

### Build Release versus Debug builds

To build packages in debug mode with debug symbols, add `cmake` args specifying the build type.

For example, to rebuild all package in debug mode use:

```bash
$ catkin build --cmake-args -DCMAKE_BUILD_TYPE=Debug
```

To build in release use:

```bash
$ catkin build --cmake-args -DCMAKE_BUILD_TYPE=Release
```

For more info see the [documentation on CMAKE_BUILD_TYPE](https://cmake.org/cmake/help/v3.5/variable/CMAKE_BUILD_TYPE.html).

<!-- ## Testing GEAR

Run this command to run both unit and integration tests to make sure GEAR is working correctly.

```
catkin test --no-deps nist_gear test_ariac
```

The above command says to run tests for the `nist_gear` and `test_ariac` package.
Unit tests (shorter, but less coverage) are in `nist_gear`.
Long running integration tests are in `test_ariac`. -->

## Run GEAR

See [Wiki | Tutorials | Interacting with GEAR](gear_interface.md) to build and use GEAR.


-------------------------------------------------
Wiki | [Home](../../README.md) | [Documentation](../documentation/documentation.md) | [Tutorials](../tutorials/tutorials.md) | [Qualifiers](../qualifiers/qualifier.md) | [Finals](../finals/finals.md)
