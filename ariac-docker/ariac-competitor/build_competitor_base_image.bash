#!/usr/bin/env bash
set -e

YELLOW='\033[0;33m'
NOCOLOR='\033[0m'

# Comment this line to rebuild using the cache
DOCKER_ARGS="--no-cache"

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

echo "${YELLOW}Build competitor base image${NOCOLOR}"

docker build ${DOCKER_ARGS} -t ariac-competitor-base-melodic:latest ${DIR}/ariac-competitor-base
