#!/usr/bin/env python

# -----------------------
#
# Node to communicate with the foxglove interface
#
# -----------------------

from matplotlib.font_manager import json_dump
import rospy
from os import listdir, rename
from os.path import isfile, join, isdir,exists
import json
import rosnode
from yaml import load, dump
from datetime import datetime
from catkin_pkg.topological_order import topological_order 
import time
from std_msgs.msg import String
from real_time_simulator.msg import Update
from real_time_simulator.msg import Data
from real_time_simulator.msg import DataMessage
import shlex
from psutil import Popen
from bs4 import BeautifulSoup
import re

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
bagFilePath = relativePathToSrc + "real_time_simulator/log/"
recentOpenedPath = relativePathToSrc + "real_time_simulator/launch/recentOpened.json"

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

        # Get recent configs
        data = {}
        with open(recentOpenedPath) as f:
            data = json.load(f)
        
        # Check if recent configs exist and send the ones that exist
        res = []
        out = {}
        validFiles = []
        files = data["files"]
        valid = True
        for file in files:
            if(exists(relativePathToSrc + file["package"] + "/launch/" + file["name"])):
                res.append(file["name"])
                validFiles.append(file)
            else:
                valid = False

        if(not valid):
            out["files"] = validFiles
            with open(recentOpenedPath, "w") as f:
                json.dump(out, f)

        msg = Data()
        msg.command = "recent_configs"
        msg.data = res
        comm_data.publish(msg) 

        #Get all config files
        onlyfiles = []

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

        for node in nodes:
            if(node[0] != "recorder"):
                v = 'rosnode kill ' + node[0]
                Popen(
                    shlex.split(v)
                )
        aList.clear()
        launch_value -= 1
        newName = datetime.now().strftime("%Y_%m_%d_%H%M%S") + ".bag"
        rename(bagFilePath + "log.bag", bagFilePath + newName)

    # Instruction to launch the system
    if(instruction.data == "launch_node"):
        print("Launch config --------")
        for node in nodes:
            v = 'rosrun ' + node[1] + ' ' + node[2] + ' ' + node[3] + ' __name:=' + node[0]
            print(v)
            node_process = Popen(
                shlex.split(v)
            )
            aList.append(node_process)

    #Instruction to save the configuration
    if(instruction.data == "save_config"):
        soup = BeautifulSoup("<launch></launch>", 'html.parser')
        for param in params:
            p = soup.new_tag("param")
            p['name'] = param[0]
            p['value'] = param[1]
            soup.launch.append(p)

        for file in paramFiles:
            group = soup.new_tag("group")
            group['ns'] = file[1]
            f = soup.new_tag("rosparam")
            f['file'] = file[0]
            group.append(f)
            soup.launch.append(group)
        soup.launch.append("")

        for node in nodes:
            n = soup.new_tag("node")
            n['name'] = node[0]
            n['pkg'] = node[1]
            n['type'] = node[2]
            if(node[3] != ""):
                n['args'] = node[3]
            if(node[4] != ""):
                n['cwd'] = node[4]
            if(node[5] != ""):
                n['output'] = node[5]
            soup.launch.append(n)
        p = relativePathToSrc + launchFiles[configFile] + "/launch/rocket_test_save.launch"
        print(p)
        
        with open(p, "w") as f:
            f.write(str(soup.prettify()))
    
    # Launch instruction
    if(instruction.data == "launch_config"):
        print("Launching config " + configFile)

        # Get recent configs
        data = {}
        with open(recentOpenedPath) as f:
            data = json.load(f)

        res = {}
        files = []
        files.append({"name":configFile, "package": launchFiles[configFile]})
        count = 0
        for file in data["files"]:
            if(file["name"] != configFile and file["package"] != "" and count < 2):
                files.append(file)
                count += 1
        res["files"] = files
        with open(recentOpenedPath, "w") as f:
            json.dump(res, f)
        

    # Launch instruction
    if(instruction.data == "stop_config"):
        print("Stop config " + configFile)


    
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

    # Select the configuration
    if(data.command == "select_config"):
        print("Setting config")
        configFile = data.data[0]
        
        p = relativePathToSrc + launchFiles[configFile] + "/launch/" + configFile
        print(p)
        
        with open(p) as f:
            soup = BeautifulSoup(f, 'html.parser')
        
        print("Extract params :")
        for param in soup.find_all('param'):
            v = 'param -> name=' + param.get('name') + ' value=' + param.get('value')
            print(v)
            params.append((param.get('name'), param.get('value')))
            comm = 'rosparam set ' + param.get('name') + ' ' + param.get('value')
            print(comm)
            param_process = Popen(
                shlex.split(comm)
            )
            aList.append(param_process)

        print("Extract param files :")
        for group in soup.find_all('group'):
            pack = group.get('ns')
            for file in group.find_all('rosparam'):
                f = file.get('file')
                m = re.search('\$\(find (.+?)\)', f)
                if(m):
                    src = m.group(1)
                    f = f.replace('$(find ' + src + ')', relativePathToSrc + src)
                    print(src)
                paramFiles.append((f, pack))
                v = 'rosparam load ' + f + ' /' + pack
                print(v)
                param_process = Popen(
                shlex.split(v)
                )
                aList.append(param_process)

        print("Extract nodes :")
        for node in soup.find_all('node'):
            args = ""
            o = node.get('output')
            if(o == None):
                o = ""
            cwd = node.get('cwd')
            if(cwd == None):
                cwd = ""
            par = node.get('args')
            if(par != None):
                m = re.search('\$\(find (.+?)\)', par)
                if(m):
                    src = m.group(1)
                    par = par.replace('$(find ' + src + ')', relativePathToSrc + src)
                args += par
            nodes.append((node.get('name'),node.get('pkg'),node.get('type'),args,cwd,o))
    
    # Update a parameter
    if(data.command == "update_param"):
        comm = 'rosparam set /environment/apogee "[0, 0, 2000]"'
        print(comm)
        param_process = Popen(
            shlex.split(comm)
        )
        aList.append(param_process)


def modifyCallback(m):
    print("Modify recieved")
    comm = 'rosparam set /' + m.config + '/' + m.parameter + ' ' + m.value
    Popen(
        shlex.split(comm)
    )
    print(m)
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
    rospy.Subscriber("updates", Update, modifyCallback)
    
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    try:
        caller()
    except rospy.ROSInterruptException:
        pass
    listener()
