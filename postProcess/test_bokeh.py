from bokeh.plotting import figure, curdoc
from bokeh.resources import CDN
from bokeh.embed import file_html
from bokeh.io import show
from bokeh.layouts import gridplot
from bokeh.models import CheckboxGroup, CustomJS, ColumnDataSource
from bokeh.models import Button, RadioGroup, FileInput, TextInput, RangeSlider
from bokeh.themes import Theme

import rospy
import rospkg
import rosbag

from real_time_simulator.msg import FSM
from real_time_simulator.msg import State
from real_time_simulator.msg import Control
from real_time_simulator.msg import Sensor
from real_time_simulator.msg import Trajectory
from real_time_simulator.msg import Waypoint

import numpy as np
import math
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R
from scipy import interpolate

import time


tStart = -1
tEnd = 40

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
controlled_z_torque = np.zeros((1,1))
time_force = np.zeros((1,1))

# Measured force from AV
measured_force = np.zeros((1,3))
measured_z_torque = np.zeros((1,1))
time_actuation = []

# Guidance optimal trajectory
target_positionZ = []
target_speedZ = []
target_prop_mass = []
time_target = [] 
thrust_target = []

select = True
select_est = True
select_force = True
select_actuation = True
select_target = True

def load_log_file(attr, old, new):
  global position 
  global speed
  global attitude 
  global omega 
  global prop_mass 
  global time_state 

  # Estimated state from Navigation
  global position_est
  global speed_est
  global attitude_est 
  global omega_est 
  global prop_mass_est
  global time_state_est 

  # Controled forces and torque
  global control_force 
  global controlled_z_torque 
  global time_force 

  # Measured force from AV
  global measured_force
  global measured_z_torque 
  global time_actuation

  # Guidance optimal trajectory
  global target_positionZ 
  global target_speedZ 
  global target_prop_mass
  global time_target
  global thrust_target 

  global select 
  global select_est 
  global select_force 
  global select_actuation 
  global select_target 
  print(new)
  bag =  rosbag.Bag(new) 

# rospack = rospkg.RosPack()
# bag = rosbag.Bag(rospack.get_path('real_time_simulator') + '/log/log.bag')

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
    controlled_z_torque = np.append(controlled_z_torque, [[msg.torque.z]]) 
    time_force = np.append(time_force, [[t.to_sec()]]) 

  for topic, msg, t in bag.read_messages(topics=['/control_measured']):
    new_force = msg.force
    measured_force = np.append(measured_force, [[new_force.x, new_force.y, new_force.z]], axis = 0)
    measured_z_torque = np.append(measured_z_torque, [[msg.torque.z]]) 
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

  control_force = control_force[1:]
  controlled_z_torque = controlled_z_torque[1:]
  time_force = time_force[1:]

  measured_force = measured_force[1:]
  measured_z_torque = measured_z_torque[1:]


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
  select_target = np.zeros_like(time_target, dtype = bool)

  update_plot()

#select_target[::20,:] = True

def update_range(attr, old, new):
  
  tStart = new[0]
  tEnd = new[1]

  global select
  global select_est
  global select_force
  global select_actuation
  global select_target

  select = np.logical_and(time_state>tStart, time_state <tEnd)
  select_est = np.logical_and(time_state_est>tStart, time_state_est <tEnd)
  select_force = np.logical_and(time_force>tStart, time_force <tEnd) 
  select_actuation = np.logical_and(time_actuation>tStart, time_actuation <tEnd) 
  select_target = np.zeros_like(time_target, dtype = bool)

  update_plot()


## ----------- Plot flight data ----------------------
doc = curdoc()
doc.theme = Theme(json={'attrs': {

    # apply defaults to Figure properties
    'Figure': {
        'toolbar_location': "Right",
        'outline_line_color': "DimGrey",
        'min_border': 10,
        'background_fill_color': "#FFFCFC",
    },

    'Line': {
        'line_width': 2,
    },

    'Axis': {
        'axis_line_color': "DimGrey",
    },
}})

f_posXY = figure()
f_posZ = figure()
f_speedXY = figure()
f_speedZ = figure()
f_attitude = figure()
f_omega = figure()
f_thrust = figure()
f_force = figure()
f_mass = figure()

source_simu = ColumnDataSource(data=dict(t=[], 
                          posX=[],
                          posY=[],
                          posZ=[],
                          speedX=[],
                          speedY=[],
                          speedZ=[],
                          attX=[],
                          attY=[],
                          attZ=[],
                          omegaX=[],
                          omegaY=[],
                          omegaZ=[],
                          mass = []))

source_nav = ColumnDataSource(data=dict(t=[], 
                          posX=[],
                          posY=[],
                          posZ=[],
                          speedX=[],
                          speedY=[],
                          speedZ=[],
                          attX=[],
                          attY=[],
                          attZ=[],
                          omegaX=[],
                          omegaY=[],
                          omegaZ=[],
                          mass = []))
source_control = ColumnDataSource(data=dict(t=[], 
                          thrust=[],
                          forceX=[],
                          forceY=[],
                          torqueZ=[]))

source_feedback = ColumnDataSource(data=dict(t=[], 
                          thrust=[],
                          forceX=[],
                          forceY=[],
                          torqueZ=[]))





f_posXY.line('t', 'posX', source=source_simu, color = "SteelBlue")
f_posXY.line('t', 'posY', source=source_simu, color = "Coral")
f_posZ.line('t', 'posZ', source=source_simu, color = "Teal")
f_speedXY.line('t', 'speedX', source=source_simu, color = "SteelBlue")
f_speedXY.line('t', 'speedY', source=source_simu, color = "Coral")
f_speedZ.line('t', 'speedZ', source=source_simu, color = "Teal")
f_attitude.line('t', 'attX', source=source_simu, color = "SteelBlue")
f_attitude.line('t', 'attY', source=source_simu, color = "Coral")
f_attitude.line('t', 'attZ', source=source_simu, color = "Teal")
f_omega.line('t', 'omegaX', source=source_simu, color = "SteelBlue")
f_omega.line('t', 'omegaY', source=source_simu, color = "Coral")
f_omega.line('t', 'omegaZ', source=source_simu, color = "Teal")
f_mass.line('t', 'mass', source=source_simu, color = "SeaGreen")

f_posXY.scatter('t', 'posX', source=source_nav, marker = "+", color = "SteelBlue")
f_posXY.scatter('t', 'posY', source=source_nav, marker = "+", color = "Coral")
f_posZ.scatter('t', 'posZ', source=source_nav, marker = "+", color = "Teal")
f_speedXY.scatter('t', 'speedX', source=source_nav, marker = "+", color = "SteelBlue")
f_speedXY.scatter('t', 'speedY', source=source_nav, marker = "+", color = "Coral")
f_speedZ.scatter('t', 'speedZ', source=source_nav, marker = "+", color = "Teal")
f_attitude.scatter('t', 'attX', source=source_nav, marker = "+", color = "SteelBlue")
f_attitude.scatter('t', 'attY', source=source_nav, marker = "+", color = "Coral")
f_attitude.scatter('t', 'attZ', source=source_nav, marker = "+", color = "Teal")
f_omega.scatter('t', 'omegaX', source=source_nav, marker = "+", color = "SteelBlue")
f_omega.scatter('t', 'omegaY', source=source_nav, marker = "+", color = "Coral")
f_omega.scatter('t', 'omegaZ', source=source_nav, marker = "+", color = "Teal")
f_mass.scatter('t', 'mass', source=source_nav, marker = "+", color = "SeaGreen")

f_thrust.line('t', 'thrust', source=source_feedback, color = "FireBrick")
f_force.line('t', 'forceX', source=source_feedback, color = "SteelBlue")
f_force.line('t', 'forceY', source=source_feedback, color = "Coral")
f_force.line('t', 'torqueZ', source=source_feedback, color = "Teal")

f_thrust.scatter('t', 'thrust', source=source_control, marker = "+", color = "FireBrick")
f_force.scatter('t', 'forceX', source=source_control, marker = "+", color = "SteelBlue")
f_force.scatter('t', 'forceY', source=source_control, marker = "+", color = "Coral")
f_force.scatter('t', 'torqueZ', source=source_control, marker = "+", color = "Teal")

LABELS = ["Simulation", "Navigation", "Horizon"]
check_plot_type = CheckboxGroup(labels=LABELS, active=[0])

file_name = TextInput()
file_name.on_change('value', load_log_file)

range_slider = RangeSlider(start=-1, end=40, value=(-1,30), step=.1, title="Time")
range_slider.on_change('value', update_range)


layout = gridplot([[f_posXY, f_posZ, f_attitude, check_plot_type], [f_speedXY, f_speedZ, f_omega, file_name], [f_mass, f_thrust, f_force, range_slider]], plot_width=350, plot_height=250)

doc.add_root(layout)

def update_plot(simu = True, nav = False, horizon = False):

  if simu:
    source_simu.data=dict(t=time_state[select], 
                          posX=position[:, 0][select],
                          posY=position[:, 1][select],
                          posZ=position[:, 2][select],
                          speedX=speed[:, 0][select],
                          speedY=speed[:, 1][select],
                          speedZ=speed[:, 2][select],
                          attX=attitude[:, 0][select],
                          attY=attitude[:, 1][select],
                          attZ=attitude[:, 2][select],
                          omegaX=omega[:, 0][select],
                          omegaY=omega[:, 1][select],
                          omegaZ=omega[:, 2][select],
                          mass = prop_mass[select])

    source_feedback.data=dict(t=time_actuation[select_actuation], 
                          thrust=measured_force[:, 2][select_actuation],
                          forceX=measured_force[:, 0][select_actuation],
                          forceY=measured_force[:, 1][select_actuation],
                          torqueZ=measured_z_torque[select_actuation])

  else:
    source_simu.data=dict(t=[], 
                          posX=[],
                          posY=[],
                          posZ=[],
                          speedX=[],
                          speedY=[],
                          speedZ=[],
                          attX=[],
                          attY=[],
                          attZ=[],
                          omegaX=[],
                          omegaY=[],
                          omegaZ=[],
                          mass = [])

    source_feedback.data=dict([], 
                          thrust=[],
                          forceX=[],
                          forceY=[],
                          torqueZ=[])

  if nav:
    point_spacing = 150
    source_nav.data=dict(t=time_state_est[select_est][::point_spacing], 
                          posX=position_est[:, 0][select_est][::point_spacing],
                          posY=position_est[:, 1][select_est][::point_spacing],
                          posZ=position_est[:, 2][select_est][::point_spacing],
                          speedX=speed_est[:, 0][select_est][::point_spacing],
                          speedY=speed_est[:, 1][select_est][::point_spacing],
                          speedZ=speed_est[:, 2][select_est][::point_spacing],
                          attX=attitude_est[:, 0][select_est][::point_spacing],
                          attY=attitude_est[:, 1][select_est][::point_spacing],
                          attZ=attitude_est[:, 2][select_est][::point_spacing],
                          omegaX=omega_est[:, 0][select_est][::point_spacing],
                          omegaY=omega_est[:, 1][select_est][::point_spacing],
                          omegaZ=omega_est[:, 2][select_est][::point_spacing],
                          mass = prop_mass_est[select_est][::point_spacing])

    source_control.data=dict(t=time_force[select_force], 
                          thrust=control_force[:, 2][select_force],
                          forceX=control_force[:, 0][select_force],
                          forceY=control_force[:, 1][select_force],
                          torqueZ=controlled_z_torque[select_force])

  else:
    source_nav.data=dict(t=[], 
                          posX=[],
                          posY=[],
                          posZ=[],
                          speedX=[],
                          speedY=[],
                          speedZ=[],
                          attX=[],
                          attY=[],
                          attZ=[],
                          omegaX=[],
                          omegaY=[],
                          omegaZ=[],
                          mass = [])
    source_control.data=dict(t=[], 
                          thrust=[],
                          forceX=[],
                          forceY=[],
                          torqueZ=[])


def check_plot_type_handler(new):
    simu = True if 0 in new else False
    nav = True if 1 in new else False
    horizon = True if 2 in new else False

    update_plot(simu, nav, horizon)

check_plot_type.on_click(check_plot_type_handler)

