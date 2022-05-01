#!/bin/bash
set -ex

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

ARIAC_VERSION=$1  # e.g. "ariac2.1.4"
GAZEBO_VERSION=$2  # e.g. "gazebo_8.4"

LIST_OF_ROS_DISTROS="melodic"

for ROS_DISTRO in ${LIST_OF_ROS_DISTROS}; do
  IMAGE_NAME=ariac-server-${ROS_DISTRO}
  TAG_NAME=ariac6-server-${ROS_DISTRO}:${ARIAC_VERSION}-${GAZEBO_VERSION}
  docker tag ${IMAGE_NAME} zeidk/${TAG_NAME}
  docker push zeidk/${TAG_NAME}
  # Also update the `latest` label to point to the latest image.
  TAG_NAME=ariac6-server-${ROS_DISTRO}:latest
  docker tag ${IMAGE_NAME} zeidk/${TAG_NAME}
  docker push zeidk/${TAG_NAME}

  IMAGE_NAME=ariac-competitor-base-${ROS_DISTRO}
  TAG_NAME=ariac6-competitor-base-${ROS_DISTRO}:${ARIAC_VERSION}
  docker tag ${IMAGE_NAME} zeidk/${TAG_NAME}
  docker push zeidk/${TAG_NAME}
  # Also update the `latest` label to point to the latest image.
  TAG_NAME=ariac6-competitor-base-${ROS_DISTRO}:latest
  docker tag ${IMAGE_NAME} zeidk/${TAG_NAME}
  docker push zeidk/${TAG_NAME}
done
