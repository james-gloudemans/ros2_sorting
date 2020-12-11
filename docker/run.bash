#!/bin/bash
NAME=ros2_sorting 
ROS_DISTRO=foxy

xhost +
docker run \
    -it \
    --rm \
    --device=/dev/dri \
    --group-add video \
    --network host \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix" \
    --volume="/home/james/projects/ros2_sorting/ros_ws/src/:/home/ros/ros_ws/src/" \
    --volume="/home/james/.gazebo/:/home/ros/.gazebo/" \
    --env="DISPLAY=${DISPLAY}" \
    --env "TERM=xterm-256color" \
    --name ${NAME} \
    "${NAME}:${ROS_DISTRO}"