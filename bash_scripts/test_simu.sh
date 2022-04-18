#!/bin/bash

# Simple script to start a simulation with constant force of 2000N. 
# When rocket runs out of fuel, the simulator stops using the commanded force and speed start to decrease

set -e
cd ~/catkin_ws
catkin_make
source ~/catkin_ws/devel/setup.sh

(sleep 5;rostopic pub /control_measured rocket_utils/Control "torque:
  x: 0.0
  y: 0.0
  z: 0.0
force:
  x: 0.0
  y: 0.0
  z: 2000.0" -r 10)&
roslaunch real_time_simulator rocket_sim.launch


