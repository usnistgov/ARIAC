#!/bin/bash
docker run -d --name ariac_container -p 6080:80 --shm-size=512m -v $PWD:/home/ubuntu/autoEval jfernandez37/ariac:ariac_latest
sleep 10
docker cp ./runTest.sh ariac_container:/home/ubuntu/ariac_ws/src/ariac/docs/automated_evaluation/
docker exec -it ariac_container bash -c ". /home/ubuntu/ariac_ws/src/ariac/docs/automated_evaluation/runTest.sh"
