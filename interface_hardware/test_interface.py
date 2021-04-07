#!/usr/bin/env python3
import rospy

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

#COMMANDS
GET_STAT = 	0x00
BOOT = 		0x01
SHUTDOWN = 	0x02
DOWNLOAD = 	0x03
TVC_MOVE = 	0x04
ABORT = 	0x05
RECOVER =	0x06
TRANSACTION = 0x07

control_pub = None
hb = None

def simu_sensor_callback(simu_sensor):
    sens_data = struct.pack("iii"+"iii"+"ii",   simu_sensor.IMU_acc.x,
                                                simu_sensor.IMU_acc.y,
                                                simu_sensor.IMU_acc.z,

                                                simu_sensor.IMU_gyro.x,
                                                simu_sensor.IMU_gyro.y,
                                                simu_sensor.IMU_gyro.z,

                                                simu_sensor.baro_height,
                                                0)

    resp = hb.send(TRANSACTION, sens_data)

    if(resp and len(resp) == 20):
        cmd_data = struct.unpack("i" + "iiii", bytes(resp))

        current_control = Control()
        current_control.force.z = cmd_data[0]

        control_pub.publish(current_control)




if __name__ == '__main__':

    hb = msv2.msv2()

    if hb.connect("/dev/ttyACM0"):
        print("connected")
    else:
        print("error")

    hb.send(BOOT, [0x00, 0x00])

    state_Pi = 1

    while state_Pi == 1:
        time.sleep(1)

        stat = hb.send(GET_STAT, [0x00, 0x00])
        if stat == -1 or stat==0:
            continue

        if(stat and len(stat) == 20):
            data = struct.unpack("HHiIiHBb", bytes(stat))
            state_Pi = data[0]


    # Init ROS
    rospy.init_node('test_interface', anonymous=True)
    
    # Publisher for commanded rocket_control 
    control_pub = rospy.Publisher("control_pub", Control, queue_size=10)

    # Subscribe to fake sensor from simulation 
    rospy.Subscriber("simu_sensor_pub", Sensor, simu_sensor_callback)
    

    
            
