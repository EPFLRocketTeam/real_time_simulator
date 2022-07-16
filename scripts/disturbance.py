#!/usr/bin/env python3

# -----------------------
#
# Node to compute parameterizable disturbations forces and torques acting on the rocket
#
# Input:
#	 - Full simulated state from integrator node:    			\rocket_state
#    - 3D force and torque from the control node:    			\control_pub
#	 - Finite state machine and time since start of simulation:	\fsm_pub
#
# Parameters:
#    - Rocket model: 		/config/rocket_parameters.yaml
#    - Environment model: 	/config/environment_parameters.yaml
#	 - Perturbations model: /config/perturbations_parameters.yaml
#
# Outputs:
#    - Aerodynamic 3D force and torque:  \rocket_aero
#
# -----------------------

from turtle import update
import rospy

import numpy as np
import math
import time
from scipy.spatial.transform import Rotation as R

from real_time_simulator.msg import Control
from real_time_simulator.msg import FSM
from real_time_simulator.msg import State


def fsm_callback(fsm):
	global current_fsm
	current_fsm.time_now = fsm.time_now
	current_fsm.state_machine = fsm.state_machine


def rocket_state_callback(state):
	global rocket_state
	rocket_state = state

def control_callback(control):
	global current_control
	current_control = control
		

def update_callback(command):
	# Handle update
	print("Update")
	print(command.config)
	print(command.parameter)
	print(command.value)
		

class Rocket:
	dry_mass = 0
	propellant_mass = 0
	dry_CM = 0

	Isp = 0
	maxThrust = 0
	minThrust = 0

	ground_altitude = 0
	groung_temperature = 0
	

	target_apogee = np.zeros(3)
	Cd = np.zeros(3)
	surface = np.zeros(3)
	length = 0
	diameter = np.zeros(3)

	def __init__(self):
		rocket_data = rospy.get_param("/rocket")
		env_data = rospy.get_param("/environment")
		
		self.dry_mass = rocket_data["dry_mass"]
		self.propellant_mass = rocket_data["propellant_mass"]
		self.dry_CM = rocket_data["dry_CM"]

		self.Isp = rocket_data["Isp"]
		self.maxThrust = rocket_data["maxThrust"]
		self.minThrust = rocket_data["minThrust"]

		self.ground_altitude = env_data["ground_altitude"]
		self.ground_temperature = env_data["temperature"]

		nStage = rocket_data["stages"]
		self.Cd = np.asarray(rocket_data["Cd"])
		self.diameter = np.asarray(rocket_data["diameters"])
		self.length = rocket_data["stage_z"][nStage-1]


		self.surface[0] = self.diameter[1]*self.length
		self.surface[1] = self.surface[0]
		self.surface[2] = self.diameter[1]*self.diameter[1]/4 * 3.14159
		
		
	def getPression(self, z):
		return 101325*np.exp(-0.00012*(z+ self.ground_altitude))

	def getDensity(self, z):
		return self.getPression(z)/(287.058*self.ground_temperature)


class Disturbance:
	wind_gust_intensity = np.zeros(2)
	wind_gust_assymetry = 0
	wind_gust_var = 0

	motor_tilt = np.zeros(2)
	plume_tilt_var = np.zeros(2)

	fins_tilt = np.zeros(3)
	drag_assymetry = np.zeros(2)

	air_density_bias = 0


	def __init__(self):
		chaos_data = rospy.get_param("/perturbation")

		self.wind_gust_intensity = np.asarray(chaos_data["wind_gust_intensity"])
		self.wind_gust_assymetry = chaos_data["wind_gust_assymetry"]
		self.wind_gust_var = chaos_data["wind_gust_var"]

		self.motor_tilt = np.asarray(chaos_data["motor_tilt"])
		self.plume_tilt_var = np.asarray(chaos_data["plume_tilt_var"])

		self.fins_tilt = np.asarray(chaos_data["fins_tilt"])
		self.drag_assymetry = np.asarray(chaos_data["drag_assymetry"])
		
		self.air_density_bias = chaos_data["air_density_bias"]


	# Compute random force and torque due to gust of wind
	def get_gust_disturbance(self, rocket, state):
		# Wind velocity is normal law with mean and std defined in YAML parameters
		wind_speed = np.random.normal(self.wind_gust_intensity, self.wind_gust_var*self.wind_gust_intensity/100) 

		rho_air = rocket.getDensity(state.pose.position.z)
		force = 0.5*rocket.Cd[0:2]*rocket.surface[0:2]*rho_air*wind_speed**2 
		force = np.append(force, 0)

		gust_position = (2*np.random.rand()-1)*self.wind_gust_assymetry*rocket.length/2
		torque = force*gust_position

		return np.array([force, torque])


	def get_thrust_disturbance(self, rocket):
		global current_control

		thrust_tilt = np.random.normal(self.motor_tilt, self.plume_tilt_var)
		r = R.from_euler("yx", thrust_tilt, degrees =True)

		control_thrust = np.zeros(3)
		control_thrust[0] = current_control.force.x
		control_thrust[1] = current_control.force.y
		control_thrust[2] = current_control.force.z

		real_thrust = r.apply(control_thrust)

		force = real_thrust - control_thrust

		torque = force*rocket.dry_CM
		torque[2] = 0

		return np.array([force, torque])












if __name__ == '__main__':

	# Create global variable
	current_fsm = FSM()
	rocket_state = State()
	current_control = Control()

	# Init ROS
	rospy.init_node('disturbance', anonymous=True)

	# Init fsm
	current_fsm.time_now = 0;
	current_fsm.state_machine = "Idle";

	# Subscribe to fsm 
	rospy.Subscriber("fsm_pub", FSM, fsm_callback)

	# Subscribe to rocket_control 
	rospy.Subscriber("control_measured", Control, control_callback)

	# Subscribe to rocket_state 
	rospy.Subscriber("rocket_state", State, rocket_state_callback)
	

	# Publisher for disturbance control
	disturbance_pub = rospy.Publisher('disturbance_pub', Control, queue_size=10)
	
	

	rocket = Rocket()
	chaos = Disturbance()

  # Disturbance rate in Hz
	rate = rospy.Rate(1)

	while not rospy.is_shutdown():

		# Thread sleep time defined by rate
		rate.sleep()

		gust_disturbance = chaos.get_gust_disturbance(rocket, rocket_state)
		thrust_disturbance = chaos.get_thrust_disturbance(rocket)

		total_disturbance = gust_disturbance+thrust_disturbance

		current_disturbance = Control()
		current_disturbance.force.x = total_disturbance[0][0]
		current_disturbance.force.y = total_disturbance[0][1]
		current_disturbance.force.z = total_disturbance[0][2]

		current_disturbance.torque.x = total_disturbance[1][0]
		current_disturbance.torque.y = total_disturbance[1][1]
		current_disturbance.torque.z = total_disturbance[1][2]

		disturbance_pub.publish(current_disturbance)