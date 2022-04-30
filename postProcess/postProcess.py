#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import rospkg

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
from scipy import interpolate

import time

import rosbag

tStart = -1
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
feedback_gimbal = np.zeros((1,2))
feedback_thrust = np.zeros((1,1))
time_gimbal = np.zeros((1,1))
feedback_torque = np.zeros((1,1))
time_cmg = np.zeros((1,1))

# Measured force from AV
measured_force = np.zeros((1,3))
time_actuation = []

# Guidance optimal trajectory
target_positionZ = []
target_speedZ = []
target_prop_mass = []
time_target = [] 
thrust_target = []

rospack = rospkg.RosPack()
bag = rosbag.Bag(rospack.get_path('real_time_simulator') + '/log/log.bag')

for topic, msg, t in bag.read_messages(topics=['/fsm_pub']):
  if msg.state_machine != "Idle":
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

for topic, msg, t in bag.read_messages(topics=['/gimbal_state_0']):
  feedback_gimbal = np.append(feedback_gimbal, [[msg.outer_angle, msg.inner_angle]], axis = 0)
  feedback_thrust = np.append(feedback_thrust, [[msg.thrust]], axis = 0)
  time_gimbal = np.append(time_gimbal, [[t.to_sec()]]) 

for topic, msg, t in bag.read_messages(topics=['/cmg_state_0']):
  feedback_torque = np.append(feedback_torque, [[msg.torque]], axis = 0)
  time_cmg = np.append(time_cmg, [[t.to_sec()]]) 

for topic, msg, t in bag.read_messages(topics=['/simu_actuator']):
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

time_target = np.array(time_target)
target_positionZ = np.array(target_positionZ)
target_speedZ = np.array(target_speedZ)
target_prop_mass = np.array(target_prop_mass)
thrust_target = np.array(thrust_target)

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

feedback_gimbal = np.rad2deg(feedback_gimbal[1:])
feedback_thrust = feedback_thrust[1:]
time_gimbal = time_gimbal[1:]

feedback_torque = feedback_torque[1:]
time_cmg = time_cmg[1:]

measured_force = measured_force[1:]


# Synchronize time
time_gimbal = time_gimbal - time_init
time_cmg = time_cmg - time_init
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
select_gimbal = np.logical_and(time_gimbal>tStart, time_gimbal <tEnd) 
select_cmg = np.logical_and(time_cmg>tStart, time_cmg <tEnd) 
select_actuation = np.logical_and(time_actuation>tStart, time_actuation <tEnd) 
select_target = np.zeros_like(time_target, dtype = bool)

#select_target[::20,:] = True
print(np.arccos(np.mean(- quaternion[:, 0][select]**2 - quaternion[:, 1][select]**2 + quaternion[:, 2][select]**2 + quaternion[:, 3][select]**2, axis = 0)))

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


l = axe[0][2].plot(time_state[select], attitude[:, 0][select], label = "X") 
l = axe[0][2].plot(time_state[select], attitude[:, 1][select], label = "Y") 
axe[0][2].axhline(y=-30, color='r', linestyle='--')
axe[0][2].axhline(y=30, color='r', linestyle='--')
axe[0][2].set_title("Euler angle [degree]")
axe[0][2].legend()

#l = axe[0][3].plot(time_state[select], quaternion[select])
l = axe[0][3].plot(time_state[select], attitude[:,2][select],  label = 'Z', color = "green")
axe[0][3].set_title("Euler angle [degree]")
axe[0][3].legend()

l = axe[1][2].plot(time_state[select], omega[:, 0][select], label = "X")
l = axe[1][2].plot(time_state[select], omega[:, 1][select], label = "Y")
axe[1][2].set_title("Angular speed [deg/s]")
axe[1][2].legend()

l = axe[1][3].plot(time_state[select], omega[:, 2][select],  label = 'Z', color = "green")
axe[1][3].set_title("Angular speed [deg/s]")
axe[1][3].legend()

l = axe[2][2].plot(time_gimbal[select_gimbal], feedback_gimbal[:, 0][select_gimbal], label = "X")
l = axe[2][2].plot(time_gimbal[select_gimbal], feedback_gimbal[:, 1][select_gimbal], label = "Y")
axe[2][2].set_title('Gimbal angle [°]')
axe[2][2].legend()

l = axe[2][3].plot(time_cmg[select_cmg], feedback_torque[select_cmg], color = "green")
axe[2][3].set_title("CMG Z torque [N.m]")

# l = axe[2][3].plot(time_gimbal[select_gimbal], z_torque[select_gimbal], label = 'Z torque [N.m]', color = "green")
# axe[2][3].legend()

l = axe[2][1].plot(time_actuation[select_actuation], measured_force[:, 2][select_actuation], label = "Z force [N]", linewidth=2, marker = "+")
axe[2][1].legend()

l = axe[2][0].plot(time_state[select], prop_mass[select], label = "Mass", linewidth=4)
axe[2][0].set_title("Propellant mass [kg]")
axe[2][0].legend()

# Plot Navigation estimated state (if needed)
if 1:
  point_spacing = 50

  l = axe[0][0].plot(time_state_est[select_est][::point_spacing], position_est[:, 0][select_est][::point_spacing], label = 'Estimated X', marker = '+', linestyle=':', color = "c")
  l = axe[0][0].plot(time_state_est[select_est][::point_spacing], position_est[:, 1][select_est][::point_spacing], label = 'Estimated Y', marker = '+', linestyle=':', color = "r")
  axe[0][0].legend()

  l = axe[0][1].plot(time_state_est[select_est][::point_spacing], position_est[:, 2][select_est][::point_spacing], label = 'Estimated Z', marker = '+', linestyle=':')
  axe[0][1].legend()

  l = axe[1][0].plot(time_state_est[select_est][::point_spacing], speed_est[:, 0][select_est][::point_spacing], label = 'Estimated X', marker = '+', linestyle=':', color = "c")
  l = axe[1][0].plot(time_state_est[select_est][::point_spacing], speed_est[:, 1][select_est][::point_spacing], label = 'Estimated Y', marker = '+', linestyle=':', color = "r")
  axe[1][0].legend()

  l = axe[1][1].plot(time_state_est[select_est], speed_est[:, 2][select_est], label = "Estimated Z")
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

  l = axe[2][1].plot(time_gimbal[select_gimbal], feedback_thrust[select_gimbal], marker = '+', linestyle=':', label = "Commanded Z force [N]")
  axe[2][1].legend()

# Plot Guidance optimal trajectory (if needed)
if 0:
  point_spacing = 10

  l = axe[0][1].plot(time_target.T[:,::point_spacing], target_positionZ.T[:,::point_spacing])

  l = axe[1][1].plot(time_target.T[:,::point_spacing], target_speedZ.T[:,::point_spacing])

  l = axe[2][1].plot(time_target.T[:,::point_spacing], thrust_target.T[:,::point_spacing])

  l = axe[2][0].plot(time_target.T[:,::point_spacing], target_prop_mass.T[:,::point_spacing])


fig.tight_layout()

# Plot speed difference to check kalman filter
if 0:
  plt.rcParams.update({'font.size': 16})
  plot2 = plt.figure(2, figsize=(15,15))

  f_speed = interpolate.interp1d(time_state_est, speed_est[:, 2], fill_value = "extrapolate")
  f_alt = interpolate.interp1d(time_state_est, position_est[:, 2], fill_value = "extrapolate")
  plt.plot(time_state[select], np.convolve(f_speed(time_state[select])- speed[:, 2][select], np.ones(20)/20, mode='same'), "+")
  #plt.plot(time_state[select], f_alt(time_state[select])- position[:, 2][select])
  plt.xlabel("Time [s]")
  plt.ylabel("Speed error [m/s]")

plt.show()

