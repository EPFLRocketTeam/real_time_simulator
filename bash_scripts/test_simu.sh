#!/bin/bash

# Simple script to start a simulation with constant force of 2000N. 
# When rocket runs out of fuel, the simulator stops using the commanded force and speed start to decrease

set -e
cd ~/catkin_ws
catkin_make
source ~/catkin_ws/devel/setup.sh

(sleep 5;rostopic pub /gimbal_command_0 rocket_utils/GimbalControl "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
outer_angle: 0.0
inner_angle: 0.0
thrust: 2000.0" -r 10)&
roslaunch real_time_simulator rocket_sim.launch


