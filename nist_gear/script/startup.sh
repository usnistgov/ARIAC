#!/bin/bash

echo "Ensuring scoring log file exists"
log_dir="$HOME/.ariac/log"
mkdir -p "$log_dir"

scoring_log_file="$log_dir/performance.log"
gazebo_log_file="$HOME/.gazebo/server-11345/default.log"

if [ ! \( -e "${scoring_log_file}" \) ] ; then
  echo "File doesn't exist - symlinking $scoring_log_file to $gazebo_log_file"
  ln -sf ~/.gazebo/server-11345/default.log $scoring_log_file
else
  echo "File already exists: $scoring_log_file"
fi
