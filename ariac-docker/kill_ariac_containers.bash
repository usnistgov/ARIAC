#!/usr/bin/env bash

# Constants.
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[0;33m'
NOCOLOR='\033[0m'

kill_matching_containers () {
  echo "Killing any running Docker containers matching '$1'..."
  docker ps -a | grep "$1" | awk '{print $1}' | xargs --no-run-if-empty docker kill
  echo -e "${GREEN}Done.${NOCOLOR}"

  sleep 1

  # It's possible that the container is not running, but still exists.
  echo "Removing any Docker containers matching '$1'..."
  docker ps -a | grep "$1" | awk '{print $1}' | xargs --no-run-if-empty docker rm
  echo -e "${GREEN}Done.${NOCOLOR}"

  sleep 1
}

kill_matching_containers "ariac-competitor-*"
kill_matching_containers "ariac-server-*"
