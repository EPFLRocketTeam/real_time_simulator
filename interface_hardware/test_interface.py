#!/usr/bin/env python3
import rospy
import rospkg

import numpy as np
import math
import serial
import time

from real_time_simulator.msg import Control
from real_time_simulator.msg import FSM
from real_time_simulator.msg import State
from real_time_simulator.msg import Sensor

import msv2
import struct

from scipy.interpolate import interp1d

#COMMANDS
GET_STAT = 	0x00
BOOT = 		0x01
SHUTDOWN = 	0x02
DOWNLOAD = 	0x03
TVC_MOVE = 	0x04
ABORT = 	0x05
RECOVER =	0x06
SENSOR_WRITE =  0x07
COMMAND_READ =  0x08
SENSOR_READ =   0x09

control_pub = None
hb = None



def fsm_recv_callback(fsm_data):
    global current_fsm
    
    current_fsm = fsm_data


def simu_sensor_callback(simu_sensor):
    global current_control

    
    acc_x = int(1000*simu_sensor.IMU_acc.x/9.81)
    acc_y = int(1000*simu_sensor.IMU_acc.y/9.81)
    acc_z = int(1000*simu_sensor.IMU_acc.z/9.81)

    gyro_x = int(1000*simu_sensor.IMU_gyro.x/9.81)
    gyro_y = int(1000*simu_sensor.IMU_gyro.y/9.81)
    gyro_z = int(1000*simu_sensor.IMU_gyro.z/9.81)

    print("sensor received: {}".format(acc_z))


    baro = int(1000*simu_sensor.baro_height)
    
    sens_data = struct.pack("iii"+"iii"+"i", acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z, baro)

    resp = hb.send(SENSOR_WRITE,  sens_data)
    
    resp = hb.send(COMMAND_READ,   sens_data)


    if(resp and len(resp) == 46):
        cmd_data = struct.unpack("i" + "iiii"+"iii"+"iii"+"H", bytes(resp))

        current_kalman = State()
        if current_fsm.state_machine == "Idle":
            current_control.force.z = 2000
        else:
            current_control.force.z = cmd_data[0]

        print("current_Control: {}".format(cmd_data[0]))

        control_pub.publish(current_control)

        current_kalman.pose.position.x = cmd_data[5]/1000.0
        current_kalman.pose.position.y = cmd_data[6]/1000.0
        current_kalman.pose.position.z = cmd_data[7]/1000.0

        current_kalman.twist.linear.x = cmd_data[8]/1000.0
        current_kalman.twist.linear.y = cmd_data[9]/1000.0
        current_kalman.twist.linear.z = cmd_data[10]/1000.0

        current_kalman.pose.orientation.w = 1.0

        
        kalman_pub.publish(current_kalman)




if __name__ == '__main__':

    hb = msv2.msv2()

    if hb.connect("/dev/ttyACM0"):
        print("connected")
    else:
        print("error")

    state_Pi = 1

    while state_Pi == 1:
        time.sleep(1)

        stat = hb.send(GET_STAT, [0x00, 0x00])
        if stat == -1 or stat==0:
            continue

        if(stat and len(stat) == 20):
            data = struct.unpack("HHiIiHBb", bytes(stat))
            state_Pi = data[0]
    
    
    hb.send(BOOT, [0x00, 0x00])

    print("initializing ros...")
    # Init ROS
    rospy.init_node('test_interface', anonymous=True)
    
    # Publisher for commanded rocket_control 
    control_pub = rospy.Publisher("control_pub", Control, queue_size=1)

    kalman_pub = rospy.Publisher("kalman_rocket_state", State, queue_size=1)

    # Publisher for measured actuator inputs
    actuator_pub = rospy.Publisher('control_measured', Control, queue_size=1)

    current_fsm = FSM()

    current_control = Control()

    measured_control = Control()

    # Subscribe to fake sensor from simulation 
    rospy.Subscriber("simu_sensor_pub", Sensor, simu_sensor_callback)

    rospy.Subscriber("fsm_pub", FSM, fsm_recv_callback)

    if rospy.get_param("/simulation") == 3:


        # Load motor thrust curve to get real thrust (for control_measured)
        rospack = rospkg.RosPack()
        thrust_curve = np.loadtxt(rospack.get_path('bellalui_gnc') + "/config/motor_file.txt")
        f_thrust = interp1d(thrust_curve[:,0], thrust_curve[:,1])
            
        rate = rospy.Rate(100)

        while not rospy.is_shutdown():
        
            # Thread sleep time defined by rate
            rate.sleep()

            if current_fsm.state_machine != "Idle" and current_fsm.state_machine != "Rail":
                real_thrust = 0.0
                #current_control.force.z = 1000
                if current_control.force.z != 0.0 and current_fsm.time_now > thrust_curve[0,0] and current_fsm.time_now < thrust_curve[-1,0]:
                    real_thrust = float(f_thrust(current_fsm.time_now))


                measured_control.force.z = real_thrust
                #print(real_thrust)
                actuator_pub.publish(measured_control)

    #rospy.spin()

