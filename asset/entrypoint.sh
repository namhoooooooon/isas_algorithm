#!/bin/bash
set -e

# setup ros environment
source "/opt/ros/noetic/setup.bash"

# Check Workspace (catkin_ws)
# CAUTION !!!
if [ ! -d /root/catkin_ws ]; then
  mkdir -p /root/catkin_ws/src
  cd /root/catkin_ws/src
  catkin_init_workspace
  cd ..
  catkin_make
fi

source "/root/catkin_ws/devel/setup.bash"