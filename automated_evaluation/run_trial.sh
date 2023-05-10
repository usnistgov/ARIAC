#!/bin/bash

if [[ ! $1 ]] ; then
    echo "Team configuration argument not passed" 
    exit 1
fi

if [[ ! $2 ]] ; then
    echo "Trial name not specified" 
    exit 1
fi


teamName=$(python3 get_team_name.py $1)

if [[ ! $teamName ]] ; then
    echo "Team name not found" 
    exit 1
fi

docker exec -it $teamName bash -c ". /home/ubuntu/scripts/run_trial.sh $1 $2"