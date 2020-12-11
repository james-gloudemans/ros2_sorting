#!/bin/bash
# Script to build the ros/gazebo docker image
REBUILD=0
while getopts 'rd' opt; do
  case $opt in
    r) REBUILD=1 ;;
    *) echo 'Error in command line parsing' >&2
      exit 1
    esac
done
shift "$(( OPTIND - 1 ))" 
if [ $# -eq 0 ] ; then
  echo 'Specifiy the ros distrib to use: e.g. melodic, noetic...'
fi 
BASE_ROS_IMAGE=osrf/ros
ROS_DISTRO=$1
BASE_DIR=$2
docker pull ${BASE_ROS_IMAGE}:${ROS_DISTRO}-desktop
NAME=ros2_sorting
UID="$(id -u $USER)"
GID="$(id -g $USER)" 
if [ "$REBUILD" -eq 1 ]; then
  docker build \
  --no-cache \
  --build-arg BASE_ROS_IMAGE=${BASE_ROS_IMAGE} \
  --build-arg ROS_DISTRO=${ROS_DISTRO} \
  -t ${NAME}:${ROS_DISTRO} \
  -f docker/Dockerfile \
  ${BASE_DIR}
else
  docker build \
  --build-arg BASE_ROS_IMAGE=${BASE_ROS_IMAGE} \
  --build-arg ROS_DISTRO=${ROS_DISTRO} \
  -t ${NAME}:${ROS_DISTRO} \
  -f docker/Dockerfile \
  ${BASE_DIR}
fi