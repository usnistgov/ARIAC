#!/bin/bash

while ! ros2 node list | grep -q '/rosbridge_websocket'; do

    sleep 1
done

./gradlew runHelp

