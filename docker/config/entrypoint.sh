#!/bin/bash
set -e 
# setup environment
source $HOME/.bashrc 
# start in home directory 
cd /home/ros/ros_ws
. /opt/ros/foxy/setup.sh
colcon build --symlink-install
. install/setup.sh
exec bash -i -c $@