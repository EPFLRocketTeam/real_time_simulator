#!/usr/bin/env python

import rospy
from tvc_simulator.msg import State
from tvc_simulator.msg import Control
from tvc_simulator.msg import Sensor

import numpy as np
import math
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R
import csv

import time
import rosbag

from scipy.interpolate import interp1d

time_end = 20

def resample(desired_time_sequence, data_sequence):
    downsampling_indices = np.linspace(0, len(data_sequence)-1, len(desired_time_sequence)).round().astype(int)
    downsampled_array = [data_sequence[ind] for ind in downsampling_indices] 
    return  downsampled_array

# -------------- Read simu data ---------------------
bag = rosbag.Bag('../log/log.bag')

time_simu = []
accelerometer = []
gyroscope = []
barometer_simu = []

for topic, msg, t in bag.read_messages(topics=['/sensor_pub']):
  time_simu.append(t.to_sec())
  accelerometer.append(np.array([msg.IMU_acc.x, msg.IMU_acc.y, msg.IMU_acc.z]))
  gyroscope.append(np.array([msg.IMU_gyro.x, msg.IMU_gyro.y, msg.IMU_gyro.z]))
  barometer_simu.append(msg.baro_height)

for topic, msg, t in bag.read_messages(topics=['/control_pub']):
  if(msg.force.z):
    time_init = t.to_sec()
    break

bag.close()

time_simu = np.array(time_simu)-time_init
select_simu = np.logical_and(time_simu>=0.2, time_simu <time_end)
time_simu = time_simu[select_simu]-0.14

accelerometer = np.array(accelerometer)[select_simu]
gyroscope = np.array(gyroscope)[select_simu]
barometer = np.array(barometer_simu)[select_simu]




# -------------- Read flight data ---------------------
raw_data = []
raw_altitude_data = []

with open('data_Wasserfallen.csv', "r") as csvfile:
  line = csv.reader(csvfile, delimiter=',', skipinitialspace=True)
  for row in line:
    raw_data.append(row)

with open("data_altitude.txt", "r") as csvfile:
  line = csv.reader(csvfile)
  for row in line:
    raw_altitude_data.append(row[0])

raw_altitude_data = np.array(raw_altitude_data)
raw_altitude_data = raw_altitude_data.astype(np.float)
time_baro = np.linspace(0, len(raw_altitude_data)*0.01, len(raw_altitude_data)) - 20.44
select_data_baro = np.logical_and(time_baro>=0, time_baro <time_end)

time_baro = time_baro[select_data_baro]
baro_data = raw_altitude_data[select_data_baro]



raw_data = np.array(raw_data)

raw_data = np.reshape(raw_data, (-1,11))
raw_data = raw_data[1:,1:]

raw_data = raw_data.astype(np.float)

time_data = raw_data[:, 0] - 1849.35
select_data = np.logical_and(time_data>=0, time_data <time_end)

time_data = time_data[select_data]

accelerometer_data = (raw_data[:, 4:7][select_data]*9.81)/1.0244
gyroscope_data = raw_data[:, 7:][select_data]


print("Simulated apogee: {} | Measured apogee: {} ".format(max(barometer_simu), max(raw_altitude_data)))
print("Average barometer error: {}".format(np.sum(baro_data-resample(time_baro, barometer))/len(baro_data)))
print("Average accZ error: {}".format(np.sum( accelerometer_data[:, 2]-resample(time_data, accelerometer[:, 2]))/len(accelerometer_data[:, 2])))


# Plot data -----------------------------------------------

fig, axe = plt.subplots(3,4, figsize=(15,10))

# Plot simulated data
l = axe[0][0].plot(time_simu, accelerometer[:, 0:2])
axe[0][0].legend(l, ('X acc [m/s^2]', 'Y acc [m/s^2]'))

l = axe[0][1].plot(time_simu, accelerometer[:, 2])
axe[0][1].legend(l, ('Z acc [m/s^2]'))

l = axe[0][2].plot(time_simu, gyroscope)
axe[0][2].legend(l, ('X gyro [rad/s]', 'Y gyro [rad/s]', 'Z gyro [rad/s]'))

l = axe[0][3].plot(time_simu, barometer, label='barometer [m]')
axe[0][3].legend()

# Plot measured data
l = axe[1][0].plot(time_data, accelerometer_data[:, 0:2])
axe[1][0].legend(l, ('X acc [m/s^2]', 'Y acc [m/s^2]'))

l = axe[1][1].plot(time_data, accelerometer_data[:, 2])
axe[1][1].legend(l, ('Z acc [m/s^2]'))

l = axe[1][2].plot(time_data, gyroscope_data)
axe[1][2].legend(l, ('X gyro [rad/s]', 'Y gyro [rad/s]', 'Z gyro [rad/s]'))

l = axe[1][3].plot(time_baro, baro_data, label='barometer [m]')
axe[1][3].legend()

# Plot error in [%]
l = axe[2][0].plot(time_data, accelerometer_data[:, 0:2]-resample(time_data, accelerometer[:, 0:2]))
axe[2][0].legend(l, ('X acc [m/s^2]', 'Y acc [m/s^2]'))

l = axe[2][1].plot(time_data, accelerometer_data[:, 2]-resample(time_data, accelerometer[:, 2]))
axe[2][1].legend(l, ('Z acc [m/s^2]'))

l = axe[2][2].plot(time_data, gyroscope_data-resample(time_data, gyroscope))
axe[2][2].legend(l, ('X gyro [rad/s]', 'Y gyro [rad/s]', 'Z gyro [rad/s]'))

l = axe[2][3].plot(time_baro, baro_data-resample(time_baro, barometer), label='barometer error [m]')
axe[2][3].legend()


fig.tight_layout()

  

plot2 = plt.figure(2, figsize=(15,15))
plt.rcParams.update({'font.size': 16})
d1 = plt.plot(time_baro, baro_data, label = "Measured data")
d2 = plt.plot(time_simu, barometer, label ="Simulated data")
plt.legend()
plt.xlabel("Time [s]")
plt.ylabel("Altitude [m]")



plt.show()