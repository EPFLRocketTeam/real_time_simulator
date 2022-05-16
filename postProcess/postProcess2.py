#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from rocket_utils.msg import FSM
from rocket_utils.msg import State
from rocket_utils.msg import Control
from rocket_utils.msg import Sensor
from rocket_utils.msg import Trajectory
from rocket_utils.msg import Waypoint

import numpy as np
import math
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R

import time 


import rosbag

tStart = 0
tEnd = 30

# Simulated state
position = np.zeros((1,3))
speed = np.zeros((1,3))
attitude = np.zeros((1,4))
omega = np.zeros((1,3))
prop_mass = np.zeros((1,1))
time_state = np.zeros((1,1))

# Estimated state from Navigation
position_est = np.zeros((1,3))
speed_est = np.zeros((1,3))
attitude_est = np.zeros((1,4))
omega_est = np.zeros((1,3))
prop_mass_est = np.zeros((1,1))
time_state_est = np.zeros((1,1))

# Controled forces and torque
control_force = np.zeros((1,3))
z_torque = np.zeros((1,1))
time_force = np.zeros((1,1))

# Measured force from AV
measured_force = np.zeros((1,3))
time_actuation = []

# Guidance optimal trajectory
target_positionZ = []
target_speedZ = []
target_prop_mass = []
time_target = [] 
thrust_target = []

bag = rosbag.Bag('../log/log.bag')

for topic, msg, t in bag.read_messages(topics=['/fsm_pub']):
  if msg.state_machine == "Rail":
    time_init = t.to_sec()
    break
  

for topic, msg, t in bag.read_messages(topics=['/rocket_state']):
    new_pos = msg.pose.position
    new_speed = msg.twist.linear
    new_attitude = msg.pose.orientation    
    new_omega = msg.twist.angular
    new_mass = msg.propeller_mass
        
    position = np.append(position, [[new_pos.x, new_pos.y, new_pos.z]], axis = 0)
    speed = np.append(speed, [[new_speed.x, new_speed.y, new_speed.z]], axis = 0)
    attitude = np.append(attitude, [[ new_attitude.x, new_attitude.y, new_attitude.z, new_attitude.w]], axis = 0)
    omega = np.append(omega, [[new_omega.x, new_omega.y, new_omega.z]], axis = 0)
    prop_mass = np.append(prop_mass, [[new_mass]])
    time_state = np.append(time_state, [[t.to_sec()]])

for topic, msg, t in bag.read_messages(topics=['/kalman_rocket_state']):
    new_pos = msg.pose.position
    new_speed = msg.twist.linear
    new_attitude = msg.pose.orientation    
    new_omega = msg.twist.angular
    new_mass = msg.propeller_mass
        
    position_est = np.append(position_est, [[new_pos.x, new_pos.y, new_pos.z]], axis = 0)
    speed_est = np.append(speed_est, [[new_speed.x, new_speed.y, new_speed.z]], axis = 0)
    attitude_est = np.append(attitude_est, [[ new_attitude.x, new_attitude.y, new_attitude.z, new_attitude.w]], axis = 0)
    omega_est = np.append(omega_est, [[new_omega.x, new_omega.y, new_omega.z]], axis = 0)
    prop_mass_est = np.append(prop_mass_est, [[new_mass]])
    time_state_est = np.append(time_state_est, [[t.to_sec()]])

for topic, msg, t in bag.read_messages(topics=['/control_pub']):
  new_force = msg.force
  control_force = np.append(control_force, [[new_force.x, new_force.y, new_force.z]], axis = 0)
  z_torque = np.append(z_torque, [[msg.torque.z]]) 
  time_force = np.append(time_force, [[t.to_sec()]]) 

for topic, msg, t in bag.read_messages(topics=['/control_measured']):
  new_force = msg.force
  measured_force = np.append(measured_force, [[new_force.x, new_force.y, new_force.z]], axis = 0)
  time_actuation = np.append(time_actuation, [[t.to_sec()]]) 

  

for topic, msg, t in bag.read_messages(topics=['/target_trajectory']):
  new_waypoint = msg.trajectory

  time_target.append([point.time for point in new_waypoint])
  target_positionZ.append([point.position.z for point in new_waypoint])
  target_speedZ.append([point.speed.z for point in new_waypoint])
  target_prop_mass.append([point.propeller_mass for point in new_waypoint])
  thrust_target.append([point.thrust for point in new_waypoint])
   
bag.close()

'''
time_target = np.array(time_target)
target_positionZ = np.array(target_positionZ)
target_speedZ = np.array(target_speedZ)
target_prop_mass = np.array(target_prop_mass)
thrust_target = np.array(thrust_target)
'''

print("Apogee: {}".format(max(position[:, 2])))

# Only keep ROS data
prop_mass = prop_mass[1:]
speed = speed[1:]
omega = omega[1:]
position = position[1:]
attitude = attitude[1:]
time_state = time_state[1:]

prop_mass_est = prop_mass_est[1:]
speed_est = speed_est[1:]
omega_est = omega_est[1:]
position_est = position_est[1:]
attitude_est = attitude_est[1:]
time_state_est = time_state_est[1:]

control_force = control_force[1:]
z_torque = z_torque[1:]
time_force = time_force[1:]

measured_force = measured_force[1:]


# Synchronize time
time_force = time_force - time_init
time_state = time_state - time_init
time_state_est = time_state_est - time_init
time_actuation = time_actuation - time_init

# Convert quaternion to euler for easier visualization
quaternion = attitude
r = R.from_quat(attitude)
attitude = r.as_euler('zyx', degrees=True)

quaternion_est = attitude_est
r = R.from_quat(attitude_est)
attitude_est = r.as_euler('zyx', degrees=True)

# Convert radians to degrees for easier visualization
omega = np.rad2deg(omega)
omega_est = np.rad2deg(omega_est)


select = np.logical_and(time_state>tStart, time_state <tEnd)
select_est = np.logical_and(time_state_est>tStart, time_state_est <tEnd)
select_force = np.logical_and(time_force>tStart, time_force <tEnd) 
select_actuation = np.logical_and(time_actuation>tStart, time_actuation <tEnd) 
#select_target = np.zeros_like(time_target, dtype = bool)

#select_target[::20,:] = True

# Plot all flight data
fig, axe = plt.subplots(3,4, figsize=(15,10))

plt.rcParams.update({'font.size': 7})

# Plot simulated data as reference
l = axe[0][0].plot(time_state[select], position[:, 0][select], label='X')
l = axe[0][0].plot(time_state[select], position[:, 1][select], label='Y')
axe[0][0].set_title("Position [m]")
axe[0][0].legend()

l = axe[0][1].plot(time_state[select], position[:, 2][select], label = 'Z [m]', linewidth=4)
axe[0][1].set_title("Position [m]")
axe[0][1].legend()

l = axe[1][0].plot(time_state[select], speed[:, 0][select], label = "X")
l = axe[1][0].plot(time_state[select], speed[:, 1][select], label = "Y")
axe[1][0].set_title("Speed [m/s]")
axe[1][0].legend()

l = axe[1][1].plot(time_state[select], speed[:, 2][select],  label = 'Z', linewidth=4)
axe[1][1].set_title("Speed [m/s]")
axe[1][1].legend()


l = axe[0][2].plot(time_state[select], attitude[:, 1][select], label = "X") 
l = axe[0][2].plot(time_state[select], attitude[:, 2][select], label = "Y") 
axe[0][2].axhline(y=-30, color='r', linestyle='--')
axe[0][2].axhline(y=30, color='r', linestyle='--')
axe[0][2].set_title("Euler angle [degree]")
axe[0][2].legend()

#l = axe[0][3].plot(time_state[select], quaternion[select])
l = axe[0][3].plot(time_state[select], attitude[:,0][select],  label = 'Z', color = "green")
axe[0][3].set_title("Euler angle [degree]")
axe[0][3].legend()

l = axe[1][2].plot(time_state[select], omega[:, 0][select], label = "X")
l = axe[1][2].plot(time_state[select], omega[:, 1][select], label = "Y")
axe[1][2].set_title("Angular speed [deg/s]")
axe[1][2].legend()

l = axe[1][3].plot(time_state[select], omega[:, 2][select],  label = 'Z', color = "green")
axe[1][3].set_title("Angular speed [deg/s]")
axe[1][3].legend()

l = axe[2][2].plot(time_force[select_force], control_force[:, 0][select_force], label = "X")
l = axe[2][2].plot(time_force[select_force], control_force[:, 1][select_force], label = "Y")
axe[2][2].set_title('Thrust force [N]')
axe[2][2].legend()

l = axe[2][3].plot(time_force[select_force], z_torque[select_force], label = 'Z torque [N.m]', color = "green")
axe[2][3].legend()

l = axe[2][1].plot(time_force[select_force], control_force[:, 2][select_force], label = "Commanded Z force [N]", linewidth=4, marker = "+")
axe[2][1].legend()

l = axe[2][1].plot(time_actuation[select_actuation], measured_force[:, 2][select_actuation], label = "Z force [N]", linewidth=2, marker = "+")
axe[2][1].legend()

l = axe[2][0].plot(time_state[select], prop_mass[select], label = "Mass", linewidth=4)
axe[2][0].set_title("Propellant mass [kg]")
axe[2][0].legend()

# Plot Navigation estimated state (if needed)
if 1:
  point_spacing = 1

  l = axe[0][0].plot(time_state_est[select_est][::point_spacing], position_est[:, 0][select_est][::point_spacing], label = 'Estimated X', marker = '+', linestyle=':', color = "c")
  l = axe[0][0].plot(time_state_est[select_est][::point_spacing], position_est[:, 1][select_est][::point_spacing], label = 'Estimated Y', marker = '+', linestyle=':', color = "r")
  axe[0][0].legend()

  l = axe[0][1].plot(time_state_est[select_est][::point_spacing], position_est[:, 2][select_est][::point_spacing], label = 'Estimated Z', marker = '+', linestyle=':')
  axe[0][1].legend()

  l = axe[1][0].plot(time_state_est[select_est][::point_spacing], speed_est[:, 0][select_est][::point_spacing], label = 'Estimated X', marker = '+', linestyle=':', color = "c")
  l = axe[1][0].plot(time_state_est[select_est][::point_spacing], speed_est[:, 1][select_est][::point_spacing], label = 'Estimated Y', marker = '+', linestyle=':', color = "r")
  axe[1][0].legend()

  l = axe[1][1].plot(time_state_est[select_est][::point_spacing], speed_est[:, 2][select_est][::point_spacing], marker = '+', linestyle=':', label = "Estimated Z")
  axe[1][1].legend()

  l = axe[0][2].plot(time_state_est[select_est][::point_spacing], attitude_est[:, 0][select_est][::point_spacing], marker = '+', linestyle=':', label = "Estimated X", color = "c")
  l = axe[0][2].plot(time_state_est[select_est][::point_spacing], attitude_est[:, 1][select_est][::point_spacing], marker = '+', linestyle=':', label = "Estimated Y", color = "r")
  axe[0][2].legend()

  l = axe[0][3].plot(time_state_est[select_est][::point_spacing], attitude_est[:, 2][select_est][::point_spacing], marker = '+', linestyle=':', label = "Estimated Z")
  axe[0][3].legend()  

  l = axe[1][2].plot(time_state_est[select_est][::point_spacing], omega_est[:, 0][select_est][::point_spacing], marker = '+', linestyle=':', label = "Estimated X", color = "c")
  l = axe[1][2].plot(time_state_est[select_est][::point_spacing], omega_est[:, 1][select_est][::point_spacing], marker = '+', linestyle=':', label = "Estimated Y", color = "r")
  axe[1][2].legend()

  l = axe[1][3].plot(time_state_est[select_est][::point_spacing], omega_est[:, 2][select_est][::point_spacing], marker = '+', linestyle=':', label = "Estimated Z")
  axe[1][3].legend()

  l = axe[2][0].plot(time_state_est[select_est][::point_spacing], prop_mass_est[select_est][::point_spacing], marker = '+', linestyle=':', label = "Estimated mass [kg]")
  axe[2][0].legend()

# Plot Guidance optimal trajectory (if needed)
if 0:
  l = axe[0][1].plot(time_target.T[:,::20], target_positionZ.T[:,::20])

  l = axe[1][1].plot(time_target.T[:,::20], target_speedZ.T[:,::20])

  l = axe[2][1].plot(time_target.T[:,::20], thrust_target.T[:,::20])

  l = axe[2][0].plot(time_target.T[:,::20], target_prop_mass.T[:,::20])


fig.tight_layout()


plt.show()

