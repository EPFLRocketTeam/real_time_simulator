#!/usr/bin/env python3

# -----------------------
#
# Node to compute aerodynamic forces and torques acting on the rocket
#
# Input:
#	 - Full simulated state from integrator node:    			\rocket_state
#    - 3D force and torque from the control node:    			\control_pub
#	 - Finite state machine and time since start of simulation:	\fsm_pub
#
# Parameters:
#    - Rocket model: 		/config/rocket_parameters.yaml
#    - Environment model: 	/config/environment_parameters.yaml
#
# Outputs:
#    - Aerodynamic 3D force and torque:  \rocket_aero
#
# -----------------------

import rospy
from rocket_utils.msg import FSM
from rocket_utils.msg import State
from rocket_utils.msg import Control
from rocket_utils.msg import Sensor
from geometry_msgs.msg import Vector3

import numpy as np
import math
from scipy.integrate import ode, solve_ivp
#import matplotlib.pyplot as plt
import time
import yaml

from aero.Rocket.Body import Body
from aero.Rocket.Fins import Fins
from aero.Rocket.Motor import Motor
from aero.Rocket.Rocket import Rocket
from aero.Rocket.Stage import Stage
from aero.Functions import Math
from aero.Functions.Math.quat2rotmat import quat2rotmat
from aero.Functions.Math.rot2anglemat import rot2anglemat
from aero.Functions.Math.normalize_vector import normalize_vector
from aero.Functions.Math.quat_evolve import quat_evolve
from aero.Functions.Models.wind_model import wind_model
from aero.Functions.Models.robert_galejs_lift import robert_galejs_lift
from aero.Functions.Models.barrowman_lift import barrowman_lift
from aero.Functions.Math.rot2quat import rot2quat
from aero.Functions.Models.stdAtmosUS import stdAtmosUS
from aero.Simulator3D import Simulator3D
from aero.Functions.Models.stdAtmos import stdAtmos



def control_callback(control):
	global current_control
	current_control = control

def rocket_state_callback(new_state):
	global current_state
	current_state = new_state

def fsm_callback(fsm):
	global current_fsm
	current_fsm.state_machine = fsm.state_machine

def init_integrator():
	# Get ROS parameters from YAML file 
	
	rocket_data = rospy.get_param("/rocket")
	env_data = rospy.get_param("/environment")

  # Rocket definition
	position_cone = rocket_data["stage_z"][1]

	cone = Body('tangent ogive', rocket_data["diameters"][0:2], rocket_data["stage_z"][0:2])

	tube = Body("cylinder", rocket_data["diameters"][2:],
                        np.array(rocket_data["stage_z"][2:])-position_cone)

	rocket_cone = Stage('Bellalui 2 nosecone', cone, 0, 0, np.array([[0, 0, 0], [0, 0, 0], [0, 0, 0]]))

	rocket_body = Stage('Bellalui 2 body', tube, rocket_data["dry_mass"], rocket_data["dry_CM"], np.diag(rocket_data["dry_I"]))

	finDefData = {'number': rocket_data["fin_n"],
                'root_chord': rocket_data["fin_root_chord"],
                'tip_chord': rocket_data["fin_tip_chord"],
                'span': rocket_data["fin_span"],
                'sweep': rocket_data["fin_sweep"],
                'thickness': rocket_data["fin_thickness"],
                'phase': 0,
                'body_top_offset': rocket_data["fin_top_offset"],
                'total_mass': 0}

	rocket_body.add_fins(finDefData)

	#rocket_body.add_motor('../Motors/M2400T.txt')
  
	main_parachute_params = [True, 23.14, 100]
	rocket_body.add_parachute(main_parachute_params)

	drogue_parachute_params = [False, 1.75, 1000]
	rocket_body.add_parachute(drogue_parachute_params)

	Bellalui_2 = Rocket()
  
	Bellalui_2.add_stage(rocket_cone)
	Bellalui_2.add_stage(rocket_body)
  
	Bellalui_2.set_propellant_mass(rocket_data["propellant_mass"])
	Bellalui_2.set_propellant_CG(rocket_data["propellant_CM"])

	Bellalui_2.add_cg_empty_rocket(rocket_data["dry_CM"])
	Bellalui_2.set_rocket_inertia(np.diag(rocket_data["dry_I"]))
	Bellalui_2.set_payload_mass(0)

	Bellalui_2.set_motor_Isp(rocket_data["Isp"])
  
	US_Atmos = stdAtmosUS(env_data["ground_altitude"], env_data["temperature"], env_data["pressure"], env_data["humidity"] )
	US_Atmos.set_wind([0,0,0])
  
	SimObj = Simulator3D(Bellalui_2, US_Atmos)
	return SimObj
		


if __name__ == '__main__':

	# Creates global variables
	current_control = Control()

	current_fsm = FSM()

	current_state = State()

	# Init ROS
	rospy.init_node('aerodynamic', anonymous=True)

	# Init global variable
	current_fsm.state_machine = "Idle"

	# Simulation object
	rocket_sim = init_integrator()

	# Subscribe to rocket state
	rospy.Subscriber("rocket_state", State, rocket_state_callback)

	# Subscribe to control law
	rospy.Subscriber("simu_actuator", Control, control_callback)

	# Subscribe to fsm 
	rospy.Subscriber("fsm_pub", FSM, fsm_callback)

	rospy.Subscriber("/wind_speed", Vector3, lambda msg:rocket_sim.Environment.set_wind([msg.x, msg.y, msg.z]))

	# Publisher for aero force
	rocket_aero_pub = rospy.Publisher('rocket_aero', Control, queue_size=10)

	rate = rospy.Rate(30) # In Hz

	while not rospy.is_shutdown():

		# Thread sleep time 
		rate.sleep()

		if current_fsm.state_machine != "Idle":
			start_time = rospy.get_time()

			rocket_sim.Compute_aero(np.array([ 	current_state.pose.position.x, current_state.pose.position.y, current_state.pose.position.z,
										current_state.twist.linear.x, current_state.twist.linear.y, current_state.twist.linear.z,
										current_state.pose.orientation.x, current_state.pose.orientation.y, current_state.pose.orientation.z, current_state.pose.orientation.w, 
										current_state.twist.angular.x, current_state.twist.angular.y, current_state.twist.angular.z,
										current_state.propeller_mass ]),
										np.array([current_control.force.x, current_control.force.y, current_control.force.z]) )
			
			rocket_aero = rocket_sim.rocket.get_aero()

			if( not np.any(np.isnan(rocket_aero)) ):
				aero_msg = Control()

				aero_msg.force.x = rocket_aero[0][0]
				aero_msg.force.y = rocket_aero[0][1]
				aero_msg.force.z = rocket_aero[0][2]

				aero_msg.torque.x = rocket_aero[1][0]
				aero_msg.torque.y = rocket_aero[1][1]
				aero_msg.torque.z = rocket_aero[1][2]

				rocket_aero_pub.publish(aero_msg)

			#rospy.loginfo(1000*(rospy.get_time()-start_time))