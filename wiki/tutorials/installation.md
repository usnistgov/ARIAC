# Installing Introduction

This page will guide you through installing the Gazebo Environment for Agile Robotics (GEAR) 2019 on your machine.

# Prerequisites

## Ubuntu Bionic
GEAR requires a machine running [Ubuntu Bionic (18.04)](http://releases.ubuntu.com/18.04/).
If you're running Ubuntu, but not sure which version then run the following command to find out.

```
cat /etc/os-release
```

You should see 18.04 Bionic Beaver in that file.

## Gazebo 9

GeAR uses [Gazebo 9](http://gazebosim.org/blog/gazebo9) to simulate the environment.
Follow [these instructions](http://gazebosim.org/tutorials?tut=install_ubuntu&cat=install) to install it.

**Note**, if you see a crash complaining about missing a symbol in an ignition library, run this to fix it

```
sudo apt update
sudo apt upgrade libignition-math4
```

## ROS Melodic

GEAR uses [ROS Melodic](http://www.ros.org/).
[Here are instructions to install ROS Melodic](http://wiki.ros.org/melodic/Installation/Ubuntu).

## Gazebo ROS Packages

GEAR uses a custom version of [gazebo_ros_pkgs](http://wiki.ros.org/gazebo_ros_pkgs).
If you are using [depth cameras](https://github.com/ros-simulation/gazebo_ros_pkgs/pull/902) or want to test the [sensor blackout agility challenge](https://bitbucket.org/osrf/ariac/wiki/2019/agility_challenges#markdown-header-sensor-blackout) then you must build this from source.
If you aren't using depth cameras or testing the sensor blackout challenge, then feel free to skip this step.
Otherwise, clone the customized version into your [catkin workspace](http://wiki.ros.org/catkin/Tutorials/create_a_workspace).

```
git clone https://github.com/osrf/ariac-gazebo_ros_pkgs -b ariac-network-melodic
```

Then build your workspace as usual.

# Uninstall Prerelease Repo

Skip this step if you never installed a prerelease version of GEAR.
If you previously installed a prerelease of `ariac3` and would only like to use stable versions of the software, then remove it by doing the following:

```
sudo rm /etc/apt/sources.list.d/gazebo-prerelease.list
sudo apt-get remove ariac3
sudo apt-get clean
sudo apt-get update
```

# Installing GEAR

Run these instructions to install the latest stable version of GEAR

```
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable bionic main" > /etc/apt/sources.list.d/gazebo-stable.list'
wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
sudo apt-get update
sudo apt-get install ariac3
```

Source the ROS folder again after installing `ariac3`

```bash
source /opt/ros/melodic/setup.bash
```

Run this command to jump into the sample environment.

```
roslaunch osrf_gear sample_environment.launch
```

You should see a Gazebo window with the 2019 workcell.

![ariac_2019_workcell.png](https://bitbucket.org/repo/pB4bBb/images/863336135-ariac_2019_workcell.png)


# Staying up to Date

Announcements about releases will be posted [here](https://discourse.ros.org/t/ariac-code-release-updates/4009).
When a new release of GEAR is available, run this to install it.

```
sudo apt-get update
sudo apt-get upgrade ariac3
```


# Next Steps

Now that you have ariac installed, see the [Interacting With Gear](./gear_interface) tutorial for how to use it.