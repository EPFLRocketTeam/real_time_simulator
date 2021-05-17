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

# Arrays with GNC data
simu_data = None
nav_data = None

tStart = -1
tEnd = 40

simu = True
nav = False
horizon = False

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

def fill_full_state(bag, topic = ""):

  msg_count = bag.get_message_count(topic)
  np_data = np.zeros((14, msg_count))
  attitude = np.zeros((msg_count,4))
  i = 0
  for _, msg, t in bag.read_messages(topics=[topic]):
      new_attitude = msg.pose.orientation    
  
      attitude[i] = np.array([new_attitude.x, new_attitude.y, new_attitude.z, new_attitude.w])

      np_data[13, i] = t.to_sec()

      np_data[0, i] = msg.pose.position.x
      np_data[1, i] = msg.pose.position.y
      np_data[2, i] = msg.pose.position.z

      np_data[3, i] = msg.twist.linear.x
      np_data[4, i] = msg.twist.linear.y
      np_data[5, i] = msg.twist.linear.z

      np_data[9, i] = msg.twist.angular.x
      np_data[10, i] = msg.twist.angular.y
      np_data[11, i] = msg.twist.angular.z

      np_data[12, i] = msg.propeller_mass

      i = i+1

  r = R.from_quat(attitude)
  attitude_eul = r.as_euler('zyx', degrees=True)
  
  np_data[6:9, :] = np.transpose(attitude_eul)
  np_data[9:12, :] = np.rad2deg(np_data[9:12, :] )

  return np_data













def load_log_file(attr, old, new):
  global simu_data
  global nav_data

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

  bag =  rosbag.Bag(new) 

# rospack = rospkg.RosPack()
# bag = rosbag.Bag(rospack.get_path('real_time_simulator') + '/log/log.bag')

  for topic, msg, t in bag.read_messages(topics=['/fsm_pub']):
    if msg.state_machine == "Rail":
      time_init = t.to_sec()
      break
  
  simu_data = fill_full_state(bag, topic = "/rocket_state")

  nav_data = fill_full_state(bag, topic = "/kalman_rocket_state")

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

  print("Apogee: {}".format(max(simu_data[2])))

  control_force = control_force[1:]
  controlled_z_torque = controlled_z_torque[1:]
  time_force = time_force[1:]

  measured_force = measured_force[1:]
  measured_z_torque = measured_z_torque[1:]


  # Synchronize time
  time_force = time_force - time_init
  simu_data[13] = simu_data[13] - time_init
  nav_data[13] = nav_data[13] - time_init
  time_actuation = time_actuation - time_init

  update_plot()

#select_target[::20,:] = True

def update_range(attr, old, new):
  global tStart
  global tEnd

  tStart = new[0]
  tEnd = new[1]

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

def update_plot():

  select = np.logical_and(simu_data[13]>tStart, simu_data[13] <tEnd)
  select_est = np.logical_and(nav_data[13]>tStart, nav_data[13] <tEnd)
  select_force = np.logical_and(time_force>tStart, time_force <tEnd) 
  select_actuation = np.logical_and(time_actuation>tStart, time_actuation <tEnd) 
  select_target = np.zeros_like(time_target, dtype = bool)

  if simu:
    
    source_simu.data = dict(t=simu_data[13][select],
                            posX=simu_data[0][select],
                            posY=simu_data[1][select],
                            posZ=simu_data[2][select],
                            speedX=simu_data[3][select],
                            speedY=simu_data[4][select],
                            speedZ=simu_data[5][select],
                            attX=simu_data[6][select],
                            attY=simu_data[7][select],
                            attZ=simu_data[8][select],
                            omegaX=simu_data[9][select],
                            omegaY=simu_data[10][select],
                            omegaZ=simu_data[11][select],
                            mass = simu_data[12][select])

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
    source_nav.data = dict(t=nav_data[13][select_est][::point_spacing],
                            posX=nav_data[0][select_est][::point_spacing],
                            posY=nav_data[1][select_est][::point_spacing],
                            posZ=nav_data[2][select_est][::point_spacing],
                            speedX=nav_data[3][select_est][::point_spacing],
                            speedY=nav_data[4][select_est][::point_spacing],
                            speedZ=nav_data[5][select_est][::point_spacing],
                            attX=nav_data[6][select_est][::point_spacing],
                            attY=nav_data[7][select_est][::point_spacing],
                            attZ=nav_data[8][select_est][::point_spacing],
                            omegaX=nav_data[9][select_est][::point_spacing],
                            omegaY=nav_data[10][select_est][::point_spacing],
                            omegaZ=nav_data[11][select_est][::point_spacing],
                            mass = nav_data[12][select_est][::point_spacing])

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
  global simu
  global nav
  global horizon

  simu = True if 0 in new else False
  nav = True if 1 in new else False
  horizon = True if 2 in new else False

  update_plot()

check_plot_type.on_click(check_plot_type_handler)

