#!/bin/bash

# generate_video.sh: A shell script to generate a video from a Gazebo log file.
#
# E.g.: ./generate_video.sh ~/ariac_qual3/qual3b/log/gazebo/state.log output.ogv
#
# Please, install the following dependencies before using the script:
#   sudo apt-get install recordmydesktop wmctrl psmisc

set -e

# Constants
BLACK_WINDOW_TIME=5
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[0;33m'
NOCOLOR='\033[0m'

# Define usage function.
usage()
{
  echo "Usage: $0 <path_to_gazebo_log_file> <output>"
  exit 1
}

is_gzclient_running()
{
  if pgrep gzclient ; then
    true
  else
    false
  fi
}


# Wait until the /gazebo/default/world_stats topic tells us that the playback
# has been paused. This event will trigger the end of the recording.
wait_until_playback_ends()
{
  echo -n "Waiting for playback to end..."
  until gz topic -e /gazebo/default/world_stats -d 1 -u | grep "paused: true" \
    > /dev/null
  do
    sleep 1
    if ! is_gzclient_running ; then
      echo 1>&2 "GZ client not running, bailing"
      return 0
    fi
  done
  echo -e "${GREEN}OK${NOCOLOR}"
}

# Call usage() function if arguments not supplied.
[[ $# -ne 2 ]] && usage

GZ_LOG_FILE=$1
OUTPUT=$2

# Sanity check: Make sure that the log file exists.
if [ ! -f $GZ_LOG_FILE ]; then
    echo "Gazebo log file [$GZ_LOG_FILE] not found!"
    exit 1
fi

# Sanity check: Make sure that the log file path is absolute (the Gazebo
# playback doesn't work well with relative paths).
if [[ "$GZ_LOG_FILE" != /* ]]; then
  echo "Please, use an absolute path for your Gazebo log path."
  exit 1
fi

# Sanity check: Make sure that catkin_find is found.
which catkin_find > /dev/null || { echo "Unable to find catkin_find."\
  "Did you source your ROS setup.bash file?" ; exit 1; }

# Sanity check: Make sure that roslaunch can find gear_playback.launch
catkin_find --share nist_gear/launch/gear_playback.launch | grep gear_playback \
  > /dev/null || { echo "Unable to find gear_playback.launch . Did you source" \
  "your ARIAC setup.bash file?" ; exit 1; }

# Sanity check: Kill any dangling Gazebo before moving forward.
killall -wq gzserver gzclient || true

# Tell gazebo client what size and place it should be
echo "[geometry]
width=1330
height=865
x=100
y=100" > ~/.gazebo/gui.ini

# Start Gazebo in playback mode (paused).
roslaunch nist_gear gear_playback.launch state_log_path:=$GZ_LOG_FILE \
  > $OUTPUT.playback_output.txt 2>&1 &

# Wait and find the Gazebo Window ID.
until wmctrl -lp | grep Gazebo > /dev/null
do
  sleep 1
done
GAZEBO_WINDOW_ID=`wmctrl -lp | grep Gazebo | cut -d" " -f 1`

if [ -z "$GAZEBO_WINDOW_ID" ]; then
  echo "Gazebo window not detected. Exiting..."
  sleep 2
  killall -w gzserver gzclient
  exit 1
fi

# Adjust the value of this constant if needed to avoid capturing a black
# screen for a long time.
sleep $BLACK_WINDOW_TIME

# Play the simulation.
echo -n "Playing back..."
gz world -p 0
echo -e "${GREEN}OK${NOCOLOR}"

# Start recording the Gazebo Window.
echo -n "Recording..."
recordmydesktop --fps=30 -x 100 -y 100 --width=1330 --height=865 --no-sound -o $OUTPUT \
  > $OUTPUT.record_output.txt 2>&1 &
echo -e "${GREEN}OK${NOCOLOR}"

# Wait until the playback ends.
wait_until_playback_ends

# Terminate Gazebo.
echo -n "Encoding video..."
killall -w gzserver gzclient recordmydesktop
echo -e "${GREEN}OK${NOCOLOR}"
