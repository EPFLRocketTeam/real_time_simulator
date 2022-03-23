#!/bin/bash

set -e
cd ~/catkin_ws
catkin_make
source ~/catkin_ws/devel/setup.sh

gnome-terminal -- roslaunch real_time_simulator startup.launch

foxglove-studio