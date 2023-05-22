#!/bin/bash

#---------------------------------------------------------
# Example usage:./run_trial.sh nist_competitor trial_name
#---------------------------------------------------------
if [[ ! $1 ]] ; then
    echo "Team configuration argument not passed" 
    exit 1
fi

teamName=$(python3 get_team_name.py $1)
# echo "Team name: $teamName"

if [[ ! $teamName ]] ; then
    echo "Team name not found" 
    exit 1
fi

# Create a folder to copy log files from docker
if [ ! -d /tmp/.ariac2023/logs/$teamName ]; then
  mkdir -p /tmp/.ariac2023/logs/$teamName;
fi


if [[ $2 ]] ; then
    echo "==== Running trial: $2"
    docker exec -it $teamName bash -c ". /home/ubuntu/scripts/run_trial.sh $1 $2"
fi

if [[ ! $2 ]] ; then
    echo "==== Running all trials from the trials directory"
    # absolute path of the current script
    trials_dir="$( cd -- "$(dirname "$0")" >/dev/null 2>&1 ; pwd -P )" # https://stackoverflow.com/a/4774063/99379
    # get each file in the trials folder
    for entry in "$trials_dir"/trials/*
    do
        # e.g., kitting.yaml
        trial_file="${entry##*/}"
        # e.g., kitting
        trial_name=${trial_file::-5}

        docker exec -it $teamName bash -c ". /home/ubuntu/scripts/run_trial.sh $1 $trial_name"
        echo "==== Copying logs to /tmp/.ariac2023/logs/$teamName"
        docker cp $teamName:/home/ubuntu/logs/$trial_name.txt /tmp/.ariac2023/logs/$teamName

    done
fi


