#!/bin/bash
docker run -d --name ariac_container -p 6080:80 --shm-size=512m -v $PWD/autoEval:/home/ubuntu/autoEval ariac:ariac_latest
sleep 10
docker cp ./runTest.sh ariac_container:/home/ubuntu
docker exec -it ariac_container bash -c ". /home/ubuntu/ariac_ws/src/ariac/automated_evaluation/runTest.sh"
