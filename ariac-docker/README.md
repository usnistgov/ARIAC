# ARIAC Automated Evaluation

This repository contains the setup that will be used to automatically evaluate teams' submission for the Agile Robotics for Industrial Automation Competition (ARIAC) hosted by the National Institute of Standards and Technology (NIST).

## Overview

The setup that is created by the code in this repository is the setup that will be used for evaluating teams' systems automatically in the Qualifiers/Finals.
There are two main components to the ARIAC competition setup: the ARIAC server, and the competitor's system.
For security and reproducibility, the ARIAC server and the competitor's system are run in separate isolated environments called containers.
Docker is used to create these containers.

## Getting the code

Clone this code repository locally:

```
mkdir -p ~/ariac_ws && cd ~/ariac_ws
git clone https://github.com/usnistgov/ariac-docker.git
cd ~/ariac_ws/ariac-docker
```

## Installing Docker

Please, follow [these instructions](https://docs.docker.com/engine/installation/linux/ubuntu/) and install `Docker CE`.

Continue to the [post-install instructions](https://docs.docker.com/engine/installation/linux/linux-postinstall/) and complete the "Manage Docker as a non-root user" section to avoid having to run the commands on this page using `sudo`.

## Fetch the ARIAC system

To prepare the ARIAC competition system (but not run it), call:

```
./pull_dockerhub_images.bash
```

This will pull a Docker "images" of the latest version of the competition server and the base competitor machine image.

These will take a while to download.

## Preparing the workspace

Team configuration files must be put into the `team_config` directory in a folder with the name of the team.

We have provided an example submission in the `team_config` directory of this repository.
You should see that there is a directory called `example_team` that has the following configuration files in it:

```
$ ls team_config/example_team/
build_team_system.bash  run_team_system.bash    team_config.yaml
```

Together these files constitute a submission.
The files are explained at https://bitbucket.org/osrf/ariac/wiki/2019/automated_evaluation
We will work with the files of the `example_team` submission for this tutorial; you can use them as a template for your own team's submission.

## Preparing a team's system

To prepare the example team's system (but not run it), call:

```
./prepare_team_system.bash example_team

# For your team you will run:
# ./prepare_team_system.bash <your_team_name>
```

This will build a Docker "image" of the example team's system built on top of the base competitor image, ready to be launched with the ARIAC competition server.

## Running a single trial

To run an example trial (in this case the trial associated with `trial_config/sample.yaml`), call:

```
./run_trial.bash example_team sample

# For your team you will run:
# ./run_trial.bash <your_team_name> <trial_name>
```

This will instantiate Docker containers from the images that were prepared earlier: one for the ARIAC competition server, and one for your team's system.
The ARIAC environment will be started using the competition configuration file associated with the trial name (i.e. `trial_config/sample.yaml`), and the user configuration file associated with the team name (i.e. `example_team/team_config.yaml`).

_Note: The team's Docker container is started before the ARIAC competition server container. As such you might see the following message before the ARIAC server with the ROS master starts:_

> ERROR: Unable to communicate with Master.

_Note: If you see the following warning, it is safe to ignore it._

>[Wrn] [RenderEngine.cc:97] Unable to create X window. Rendering will be disabled

Once the trial has finished (because your system completed the trial, because you made a call to the `/ariac/end_competition` service, or because time ran out), the logs from the trial will be available in the `logs` directory that has now been created locally.
In the above invocation, the example code will end the competition after ~20 seconds (errors being printed to the terminal as the trial shuts down are expected).

## Reviewing the results of a trial

### Reviewing the trial performance

Once the behavior observed when playing back the trial's log file looks correct, you should then check the completion score.
To do so, open the relevant `performance.log` file (e.g. `logs/example_team/sample/performance.log`) and check the score output at the end of the file: it lists the scores for each order.

```
$ tail logs/example_team/sample/performance.log -n 25
(1518553810 6169392) [Dbg] [ROSAriacTaskManagerPlugin.cc:492] Sim time: 22
(1518553810 675725351) [Dbg] [ROSAriacTaskManagerPlugin.cc:717] Handle end service called
(1518553810 676916277) [Dbg] [ROSAriacTaskManagerPlugin.cc:579] End of trial. Final score: 0
Score breakdown:
<game_score>
Total game score: [0]
Total process time: [10.433]
Product travel time: [0]
<order_score order_0>
Total order score: [0]
Time taken: [10.432]
Complete: [false]
<shipment_score order_0_shipment_0>
Completion score: [0]
Complete: [false]
Submitted: [false]
Product presence score: [0]
All products bonus: [0]
Product pose score: [0]
</shipment_score>

</order_score>

</game_score>
```

In this example the score is 0 because the example team system does not actually complete any orders.

The general output structure of the `logs` directory is:

```
logs
└── example_team  # team name
    └── sample  # trial name
        ├── gazebo
        │   └── state.log  # gazebo state log file
        ├── generated  # the specific files generated by gear.py based on team/trial config files
        │   ├── gear.launch
        │   ├── gear.urdf.xacro
        │   └── gear.world
        ├── performance.log  # scoring log file
        ├── ros  # ROS logs copied from ARIAC server container
        │   └── rosout.log
        └── ros-competitor  # ROS logs copied from competitor container
            └── example_node-1-stdout.log
```

Additionally, there may be a `video` directory containing a video produced from playback of the simulation state log file recorded during the trial(s).
The following properties are relevant:
- Logs playback at a slower speed and therefore a 5 minute simulation may result in a 10-15 minute video. The simulation time is displayed in the bottom right.
- Simulation time may jump forward a number of seconds if there is a length of time where no movement is detected in the simulation.
- The log stops at the last time motion occurred, so log files may be shorter than expected if there is no motion.

### Playing back the simulation

To play-back a specific trial's log file, you must have ARIAC installed on your machine, and then you can call:

```
roslaunch osrf_gear gear_playback.launch state_log_path:=`pwd`/logs/example_team/sample/gazebo/state.log
```

You should see the ARIAC environment start up with parts in the bins, and the robot be controlled briefly by the example code.

*Note: this is currently only possible for user accounts with user ID of 1000.*

## Running all trials

_Only one trial config file is provided at the moment; this command will be more useful in the future._

To run all trials listed in the `trial_config` directory, call:

```
./run_all_trials.bash example_team

# For your team you will run:
# ./run_all_trials.bash <your_team_name>
```

This will run each of the trials sequentially in an automated fashion.
This is the invocation that will be used to test submissions for the Finals: your system will not be provided with any information about the trial number or the conditions of the trial.
If your system performs correctly with this invocation, regardless of the set of configuration files in the `trial_config` directory, you're ready for the competition.

## Development tips

### Keeping the competition setup software up to date

New releases of the ARIAC software will be accompanied by new releases of the Docker images, so that the latest ARIAC version is installed in both the ARIAC competition server image and the base competitor image.
Whenever there is a new release of the ARIAC software, or whenever you are informed of any other changes to the competition system setup, you will have to run `git pull` to get any recent modifications to the competition system setup, and re-run all scripts in order for the changes to take effect:

```
cd ~/ariac_ws/ariac-docker
# Get any changes to the scripts in this repo.
git pull
# Get the most recent Docker images.
./pull_dockerhub_images.bash
# Re-build your team's Docker image using the latest base competitor image.
./prepare_team_system.bash <your_team_name>
```

### Stopping the competition/containers

If during your development you need to kill the ARIAC server/competitor containers, you can do so with:

```
./kill_ariac_containers.bash
```

This will kill and remove all ARIAC containers.

### Utilizing the Docker cache to when re-building the competitor system

By default, runnng `./build_team_system.bash <your_team_name>` will re-build the image from scratch.
During development you may find it useful to call `./prepare_team_system.bash <your_team_name> --use-cache` to re-use already-built image in the Docker cache if appropriate.
However, new versions of packages may have been released since the images in the cache were built, and this will not trigger images to be re-built. Therefore you must not use this option when testing your finalized system for submission.

### Investigating build issues

If you are having difficulties installing your team's system with the `prepare_team_system` script, you can open a terminal in a clean competitor container (before the script has been run) and see which commands you need to type manually to get your system installed.

First, run:

```
docker run -it --rm --name ariac-competitor-clean-system ariac/ariac3-competitor-base-melodic:latest
```

This will start a container with the state immediately before trying to run your `build_team_system` script.
From inside this container, you can type all of the commands you need to install your code (you do not need to use `sudo`), then run `history` to get a list of the commands that you typed: that will be a good starting point for your `build_team_system` script.
You may need to modify it slightly e.g. by adding `-y` to commands that otherwise prompt for user input, such as `apt-get install -y ros-melodic-moveit-core`.

Type `exit` to stop the container.

### Investigating the contents of a running competitor container

Once your team's system has been successfully installed in the competitor container, if you are having difficulties *running* your team's system, you can open a terminal in the container that has your system installed with:

```
docker run -it --rm --name ariac-competitor-system ariac-competitor-<your_team_name>
# e.g. for example_team:
# docker run -it --rm --name ariac-competitor-system ariac-competitor-example_team
```

Inside the container you can look around with, for example:

```
ls ~/my_team_ws
```

Type `exit` to stop the container.
