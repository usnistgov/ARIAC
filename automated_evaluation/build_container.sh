#!/bin/bash

if [[ ! $1 ]] ; then
    echo "Team configuration argument not passed" 
    exit 1
fi

if [[ ! $2 ]] ; then
    echo "Port number argument not passed" 
    exit 1
fi


teamName=$(python3 get_team_name.py $1)

if [[ ! $teamName ]] ; then
    echo "Team name not found" 
    exit 1
fi

# Start the container
docker run -d --name $teamName -p $2:80 --shm-size=4gb nistariac/ariac2023:latest

# Copy scripts directory and yaml file
docker cp ./scripts/ $teamName:/home/ubuntu
docker cp ./competitor_build_scripts/ $teamName:/home/ubuntu
docker cp ./trials/ $teamName:/home/ubuntu
docker cp ./$1.yaml $teamName:/home/ubuntu/scripts


# Run build script
docker exec -it $teamName bash -c ". /home/ubuntu/scripts/build_environment.sh $1"


