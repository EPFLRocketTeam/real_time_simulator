#!/usr/bin/env python3
import rospy
import rospkg

import numpy as np
import math
import time

from real_time_simulator.msg import Control
from real_time_simulator.msg import FSM
from real_time_simulator.msg import State
from real_time_simulator.msg import Sensor

import can
import struct

from scipy.interpolate import interp1d

# DATA ID
DATA_ID_PRESSURE        = 0 
DATA_ID_ACCELERATION_X  = 1 
DATA_ID_ACCELERATION_Y  = 2 
DATA_ID_ACCELERATION_Z  = 3 
DATA_ID_GYRO_X          = 4 
DATA_ID_GYRO_Y          = 5 
DATA_ID_GYRO_Z          = 6 

DATA_ID_GPS_HDOP        = 7 
DATA_ID_GPS_LAT         = 8 
DATA_ID_GPS_LONG        = 9 
DATA_ID_GPS_ALTITUDE    = 10
DATA_ID_GPS_SATS        = 11

DATA_ID_TEMPERATURE     = 12
DATA_ID_CALIB_PRESSURE  = 13

DATA_ID_AB_STATE        = 16
DATA_ID_AB_INC          = 17
DATA_ID_AB_AIRSPEED     = 18
DATA_ID_AB_ALT          = 19

DATA_ID_KALMAN_STATE    = 38
DATA_ID_KALMAN_X        = 40
DATA_ID_KALMAN_Y        = 41
DATA_ID_KALMAN_Z        = 42
DATA_ID_KALMAN_VX       = 43
DATA_ID_KALMAN_VY       = 44
DATA_ID_KALMAN_VZ       = 45
DATA_ID_KALMAN_YAW      = 46
DATA_ID_KALMAN_PITCH    = 47
DATA_ID_KALMAN_ROLL     = 48
DATA_ID_ALTITUDE        = 49
DATA_ID_COMMAND         = 80
 
DATA_ID_PRESS_1         = 85
DATA_ID_PRESS_2         = 86
DATA_ID_TEMP_1          = 87
DATA_ID_TEMP_2          = 88
DATA_ID_TEMP_3          = 89
DATA_ID_STATUS          = 90
DATA_ID_MOTOR_POS       = 91
DATA_ID_VANE_POS_1      = 92
DATA_ID_VANE_POS_2      = 93
DATA_ID_VANE_POS_3      = 94
DATA_ID_VANE_POS_4      = 95
 
DATA_ID_TVC_COMMAND     = 100
DATA_ID_THRUST_CMD      = 101
DATA_ID_VANE_CMD_1      = 102
DATA_ID_VANE_CMD_2      = 103
DATA_ID_VANE_CMD_3      = 104
DATA_ID_VANE_CMD_4      = 105
 
DATA_ID_TVC_HEARTBEAT   = 106

T2P_CONVERSION = 76.233


TVC_CMD_BOOT = 1
TVC_CMD_SHUTDOWN = 2
TVC_CMD_ABORT = 3

STATE_IDLE = 0
STATE_BOOT = 1
STATE_COMPUTE = 2
STATE_SHUTDOWN = 3
STATE_ABORT = 4


bus = None

current_fsm = FSM()
current_control = Control()
current_kalman = State()


def send_message(data_id, data, timestamp=0):
    b_data = struct.pack(">i", data)
    b_data_id = struct.pack("b", data_id)
    b_timestamp = struct.pack(">i", timestamp)
    send_msg = can.Message(data=b_data+b_data_id+b_timestamp[0:3])
    bus.send(send_msg) 


def recv_message(timeout=0.01):
    recv_msg = bus.recv(timeout)
    if(recv_msg is not None and len(recv_msg.data) == 8):
        data_id = recv_msg.data[4]
        data = struct.unpack(">i", recv_msg.data[0:4])[0]
        timestamp = struct.unpack(">i", b'\0'+recv_msg.data[5:8])[0]
        return {"valid" : True, "id" : data_id, "data" : data, "timestamp" : timestamp}
    return {"valid" : False, "id" : 0, "data" : 0, "timestamp" : 0}



def fsm_recv_callback(fsm_data):
    global current_fsm
    
    current_fsm = fsm_data


def simu_sensor_callback(simu_sensor):
    global current_control

    acc_x = round(1000*simu_sensor.IMU_acc.x/9.81)
    acc_y = round(1000*simu_sensor.IMU_acc.y/9.81)
    acc_z = round(1000*simu_sensor.IMU_acc.z/9.81)

    gyro_x = round(1000*simu_sensor.IMU_gyro.x)
    gyro_y = round(1000*simu_sensor.IMU_gyro.y)
    gyro_z = round(1000*simu_sensor.IMU_gyro.z)

    baro = int(1000*simu_sensor.baro_height)

    cc_press = int(measured_control.force.z*T2P_CONVERSION)

    send_message(DATA_ID_ACCELERATION_X, acc_x)
    send_message(DATA_ID_ACCELERATION_Y, acc_y)
    send_message(DATA_ID_ACCELERATION_Z, acc_z)

    send_message(DATA_ID_GYRO_X, gyro_x)
    send_message(DATA_ID_GYRO_Y, gyro_y)
    send_message(DATA_ID_GYRO_Z, gyro_z)

    send_message(DATA_ID_ALTITUDE, baro)

    send_message(DATA_ID_PRESS_2, cc_press)




if __name__ == '__main__':

    THROTTLING = rospy.get_param("/rocket/throttling")

    bus = can.interface.Bus(interface='pcan', channel='PCAN_USBBUS1', bitrate=250000, state=can.bus.BusState.ACTIVE)

    if bus is None:
        exit(1)


    send_message(DATA_ID_TVC_COMMAND, TVC_CMD_BOOT)

    while(1):
        msg = recv_message()
        if msg['valid'] and msg['id'] == DATA_ID_TVC_HEARTBEAT and msg['data'] == STATE_COMPUTE:
            break

    #init ros node
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

    rospack = rospkg.RosPack()
    
    thrust_curve = np.loadtxt(rospack.get_path("real_time_simulator") + "/config/thrust_curve/motor_file.txt")
    f_thrust = interp1d(thrust_curve[:,0], thrust_curve[:,1])
        
    #variables
    new_thrust = False
    real_thrust = rospy.get_param("/rocket/maxThrust")[2]

    rate = rospy.Rate(50) #PP heartbeat

    while not rospy.is_shutdown():

        #poll can messages
        while(1):
            msg = recv_message()
            if not msg['valid']:
                break
            if msg['id'] == DATA_ID_THRUST_CMD:
                current_control.force.z = msg['data']
                new_thrust = True
            if msg['id'] == DATA_ID_KALMAN_X:
                current_kalman.pose.position.x = msg['data']/1000.0
            if msg['id'] == DATA_ID_KALMAN_Y:
                current_kalman.pose.position.y = msg['data']/1000.0
            if msg['id'] == DATA_ID_KALMAN_Z:
                current_kalman.pose.position.z = msg['data']/1000.0
            if msg['id'] == DATA_ID_KALMAN_VX:
                current_kalman.twist.linear.x = msg['data']/1000.0
            if msg['id'] == DATA_ID_KALMAN_VY:
                current_kalman.twist.linear.y = msg['data']/1000.0
            if msg['id'] == DATA_ID_KALMAN_VZ:
                current_kalman.twist.linear.z = msg['data']/1000.0


        if(new_thrust):
            new_thrust = False
            control_pub.publish(current_control)

        current_kalman.pose.orientation.w = 1
        kalman_pub.publish(current_kalman)

        if THROTTLING:
            real_thrust = current_control.force.z

        else:
            if current_fsm.time_now >= thrust_curve[0,0] and current_fsm.time_now <= thrust_curve[-1,0]:

                real_thrust = float(f_thrust(current_fsm.time_now))

                if current_fsm.state_machine != "Idle" and current_fsm.state_machine != "Rail":
                    if current_control.force.z == 0.0:
                        real_thrust = 0.0

        measured_control.force.z = real_thrust
        actuator_pub.publish(measured_control)


        rate.sleep()

        
    



