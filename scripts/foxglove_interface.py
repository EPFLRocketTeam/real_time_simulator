#!/usr/bin/env python

# -----------------------
#
# Node to communicate with the foxglove interface
#
# -----------------------

import rospy
import json
from os import listdir, rename
from os.path import isfile, join, isdir
import rosnode
from yaml import load, dump
from datetime import datetime
from catkin_pkg.topological_order import topological_order 
import time
from std_msgs.msg import String
from real_time_simulator.msg import Update
from real_time_simulator.msg import Data
import shlex
from psutil import Popen

# List of all launched processes
aList = []
# List of parameters
params = []
# List of parameter files
paramFiles = []
# List of nodes
nodes = []
# Map from launch file (.launch) to package
launchFiles = {}
# Test value
test_value = 0
# Counter to limit the number of launches
launch_value = 0
# Config file choosed for the simulation
configFile = ""


# Internal constants regarding ros path
relativePathToSrc = "../../../src/"
configsPath = relativePathToSrc + "real_time_simulator/launch/configs/"
bagFilePath = relativePathToSrc + "real_time_simulator/log/"

# Callback for the instruction topic
def instruction_callback(instruction):
    global launch_value
    global test_value
    global launchFiles
    print("Instruction recieved")
    
    # Instruction to start the simulation
    if(instruction.data == "launch"):
        # TODO : Probably to replace with a state machine : {Idle, Launched, ...?}
        if(launch_value < 1):
            print("Launch")
            launch_value += 1
            command = String()
            comm_pub.publish(command)
            time.sleep(0.5)
    
    # Instruction to reset the simulation
    if(instruction.data == "reset"):
        print("Reset simulation ---------")

    # Instruction to stop the simulation
    if(instruction.data == "stop"):
        print("Stop simulation ----------")

    # Instruction to list all ROS packages
    if(instruction.data == "get_packages"):
        res = topological_order(relativePathToSrc)
        rosnode.get_node_names
        print("--------- Package list ---------")
        print(res)
        print("List of names : ")
        for elem in res:
            print(elem[0])
    
    # Instruction to get all config that can be launched
    if(instruction.data == "get_configs"):
        # Get all config files
        print("Get config ------------")
        onlyfiles = [f for f in listdir(configsPath) if isfile(join(configsPath, f))]

        # Get all Launch files
        res = topological_order(relativePathToSrc)
        launchFiles = {}
        for elem in res:
            path = relativePathToSrc + elem[0] + "/launch/"
            if(isdir(path)):
                temp = [f for f in listdir(path) if isfile(join(path, f)) and "rocket_" in f]
                onlyfiles = onlyfiles + temp
                for file in temp:
                    launchFiles[file] = elem[0]

        # Give back the results
        msg = Data()
        msg.command = "configs"
        msg.data = onlyfiles
        comm_data.publish(msg) 

    # Instuction to stop nodes
    if(instruction.data == "stop_node"):
        print("Stopping nodes --------")
        for p in aList:
            p.terminate()
        aList.clear()
        launch_value -= 1
        newName = datetime.now().strftime("%Y_%m_%d_%H%M%S") + ".bag"
        rename(bagFilePath + "log.bag", bagFilePath + newName)
    
    # Instruction to launch the system
    if(instruction.data == "launch_node"):
        print("Launch config --------")
        if(".launch" in configFile):
            process = Popen(
                shlex.split('roslaunch ' + launchFiles[configFile] + ' ' + configFile)
            )
            aList.append(process)
        else:
            data = {}
            with open(configsPath + configFile) as f:
                data = json.load(f)
            
            p = data["listParam"]
            pf = data["listParamFiles"]
            n = data["listNodes"]
            for param in p:
                params.append((param["name"], param["value"]))
                param_process = Popen(
                    shlex.split('rosparam set ' + param["name"] + ' ' + param["value"])
                )
                aList.append(param_process)

            for param in pf:
                paramFiles.append((param["file"], param["param"]))
                param_process = Popen(
                    shlex.split('rosparam load ' + param["file"] + ' ' + param["param"])
                )
                aList.append(param_process)

            for node in n:
                nodes.append((node["package"], node["file"]))
                node_process = Popen(
                    shlex.split('rosrun ' + node["package"] + ' ' + node["file"])
                )
                aList.append(node_process)
    
    # Instruction to execute the test code (for dev, when testing and experimenting with new concepts)
    if(instruction.data == "test"):
        print("--- test ---")
        test_value += 1
        msg = String()
        msg.data = "Message arrived" + str(test_value)
        test_pub.publish(msg)
        time.sleep(0.5)

# Callback for the data topic
def dataCallBack(data):
    global configFile
    print("data recieved--------------")
    print(data)
    if(data.command == "select_config"):
        print("Setting config")
        configFile = data.data[0]


# Initialises all publishers
def caller():
    # Publisher for test topic
    global test_pub
    test_pub = rospy.Publisher('tests', String, queue_size=10)

    # Publisher for commands topic
    global comm_pub
    comm_pub = rospy.Publisher('commands', String, queue_size=10)

    # Publisher for data passing
    global comm_data
    comm_data = rospy.Publisher('data', Data, queue_size=10)

# Initialises all listened topics and give their callback
def listener():
    
    rospy.init_node('foxglove_interface', anonymous=True)

    rospy.Subscriber("instructions", String, instruction_callback)
    rospy.Subscriber("data", Data, dataCallBack)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    try:
        caller()
    except rospy.ROSInterruptException:
        pass
    listener()
