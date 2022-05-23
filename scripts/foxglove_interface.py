#!/usr/bin/env python

# -----------------------
#
# Node to communicate with the foxglove interface
#
# -----------------------

from matplotlib.pyplot import get
from numpy import empty
import rospy
from os import listdir, rename, kill, remove
from os.path import isfile, join, isdir,exists
import json
from datetime import datetime
from catkin_pkg.topological_order import topological_order 
import time
from std_msgs.msg import String
from real_time_simulator.msg import Update, FoxgloveDataMessage
import shlex
from psutil import Popen
from bs4 import BeautifulSoup
import re
import signal
from enum import Enum
import ruamel.yaml
from visualization_msgs.msg import Marker, MarkerArray
import geometry_msgs
import math
import ast 
# --------  Parameters  ---------------
# - 1

# List of all launched processes
listSubProcesses = []

# Map from launch file (.launch) to package
launchFiles = {}

# - 2

# Config file choosed for the simulation
configFile = ""

# Recent configs
recentConfigs = []
allConfigs = []

# - 3

# List of parameters
params = []

# List of parameter files
paramFiles = []

# List of nodes
nodes = []

# List of parameter files prefix
paramFilePrefixes = []

# List of parameters that have been modified
modifiedParameters = {}

# - 4
speed = -1
direction = -1
# - 5


# Test value
test_value = 0
# Counter to limit the number of launches

class SimulatorState(Enum):
    selection = 1
    configs = 2
    preLaunch = 3
    launching = 4
    launched = 5
    simulation = 6

launch_value = SimulatorState.selection

# Internal constants regarding ros path
relativePathToSrc = "../../../src/"
bagFilePath = relativePathToSrc + "real_time_simulator/log/"
recentOpenedPath = relativePathToSrc + "real_time_simulator/launch/recentOpened.json"


""" Kill all ROS nodes from the launch file

"""
def stop_simulation():

    global listSubProcesses
    global launch_value

    for p in listSubProcesses:
            kill(p.pid,signal.SIGINT)
        
    listSubProcesses.clear()
    launch_value = SimulatorState.preLaunch

    # Rename the bag file to be unique
    while(not exists(bagFilePath + "log.bag")):
        pass
    
    newName = datetime.now().strftime("%Y_%m_%d_%H%M%S") + ".bag"
    rename(bagFilePath + "log.bag", bagFilePath + newName)


""" Launch all the ROS nodes from the launch file

"""
def launch_nodes():
    global launch_value
    launch_value = SimulatorState.launching

    for node in nodes:
        v = 'rosrun ' + node[1] + ' ' + node[2] + ' ' + node[3] + ' __name:=' + node[0]
        node_process = Popen(
            shlex.split(v)
        )
        listSubProcesses.append(node_process)

    # Set default values for wind speed and wind direction
    global speed
    global direction
    env_data = rospy.get_param("/environment")
    speed = env_data["wind_speed"]
    direction = env_data["wind_direction"]
    launch_value = SimulatorState.launched


""" Clear all parameters that have been loaded in ROS from a launch file

"""
def clearParameters():
    global params
    global paramFiles
    global nodes
    global paramFilePrefixes
    global modifiedParameters
    # Clear parameters

    for param in params:
        comm = "rosparam delete /" + param[0]
        Popen(
            shlex.split(comm)
        )
        
    for prefix in paramFilePrefixes:
        comm = "rosparam delete /" + prefix
        Popen(
            shlex.split(comm)
        )

    params = []
    paramFiles = []
    nodes = []
    paramFilePrefixes = []
    modifiedParameters = []


""" Gets all configuration files containing a certain prefix and send them to foxglove

"""
def getConfigs():
    global launchFiles
    global recentConfigs
    global allConfigs

    # Get recent configs
    data = {}
    launchFiles = {}
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

    recentConfigs = res
    msg = FoxgloveDataMessage()
    msg.command = "recent_configs"
    msg.data = res
    comm_data.publish(msg) 

    #Get all config files
    onlyfiles = []

    res = topological_order(relativePathToSrc)
    for elem in res:
        path = relativePathToSrc + elem[0] + "/launch/"
        if(isdir(path)):
            temp = [f for f in listdir(path) if isfile(join(path, f)) and "_SIL" in f]
            onlyfiles = onlyfiles + temp
            for file in temp:
                launchFiles[file] = elem[0]

    allConfigs = onlyfiles
    # Give back the results
    msg = FoxgloveDataMessage()
    msg.command = "configs"
    msg.data = onlyfiles
    comm_data.publish(msg)



# Callback for the instruction topic
""" Method handling the instruction messages from instruction topic

:param instruction message recieved from foxglove containing the instruction
:type instruction message type: {string data}
"""
def instruction_callback(instruction):
    global launch_value
    global test_value
    global launchFiles
    print(instruction)
    # 1 -----------------------------------------------------

    # Instruction to get all config that can be launched
    if(instruction.data == "get_configs"):
        # Get all config files
        print("Get config")
        getConfigs()
        launch_value = SimulatorState.configs

    # Instruction to go back to home menu
    if(instruction.data == "clear_configs"):
        global recentConfigs
        global allConfigs
        recentConfigs = []
        allConfigs = []
        launch_value = SimulatorState.selection

    # 2 -----------------------------------------------------

    # Instruction to stop the simulation
    if(instruction.data == "clear_parameters"):
        print("Clear parameters")
        launch_value = SimulatorState.selection
        clearParameters()
        getConfigs()
        launch_value = SimulatorState.configs

     
    if(instruction.data == "save_parameters"):
        print("Saving parameters")
        for file in paramFiles:
            if(file[1] in modifiedParameters):
                f = open(file[0], "r")
                tmp = f.read()
                f.close()
                yaml = ruamel.yaml.YAML()
                yaml.preserve_quotes = True
                par = yaml.load(tmp)
                changed = modifiedParameters[file[1]]
                
                for p in changed:
                    par[p] = rospy.get_param(file[1] + "/" + p)
                f = open(file[0], "w")
                yaml.dump(par, f)
                f.close()
        modifiedParameters.clear()
    # 3 -----------------------------------------------------

    # Close config instruction
    if(instruction.data == "close_config"):
        print("Stop config ")
        launch_value = SimulatorState.selection
        # Clear parameters
        clearParameters()

    # Instruction to launch the system
    if(instruction.data == "launch_nodes"):
        print("Launch config --------")
        launch_nodes()

    # 4 -----------------------------------------------------
    
    # Instruction to start the simulation
    if(instruction.data == "launch_simulation"):
        if(launch_value is SimulatorState.launched):
            print("Launch")
            launch_value = SimulatorState.simulation
            command = String()
            comm_pub.publish(command)
            time.sleep(0.5)

    # Instuction to stop nodes
    if(instruction.data == "stop_nodes"):
        print("Stopping nodes")
        for p in listSubProcesses:
            kill(p.pid,signal.SIGINT)
        
        listSubProcesses.clear()

        while(not exists(bagFilePath + "log.bag")):
            pass
        remove(bagFilePath + "log.bag")
        launch_value = SimulatorState.preLaunch

    # 5 -----------------------------------------------------

    # Instuction to stop nodes
    if(instruction.data == "stop_simulation"):
        print("Stopping Simulation")
        stop_simulation()
    
    # Instruction to reset the simulation
    if(instruction.data == "restart_simulation"):
        print("Reset simulation")
        # TODO : Implement (kill all nodes and recreate them)
        stop_simulation()
        launch_nodes()

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
        
        with open(p, "w") as f:
            f.write(str(soup.prettify()))
    
    
    # Instruction to execute the test code (for dev, when testing and experimenting with new concepts)
    if(instruction.data == "test"):
        print("--- test ---")
        test_value += 1
        msg = String()
        msg.data = "Message arrived" + str(test_value)
        test_pub.publish(msg)
        time.sleep(0.5)


""" Method handling the data messages from data topic

:param data message recieved from foxglove containing data
:type data message type: {string command, string[] data}
"""
def dataCallBack(data):
    global launch_value
    global configFile
    print("data recieved")

    # 1 -----------------------------------------------------

    # Select the configuration
    if(data.command == "select_config"):
        print("Setting config")
        launch_value = SimulatorState.preLaunch
        configFile = data.data[0]
        
        p = relativePathToSrc + launchFiles[configFile] + "/launch/" + configFile
        
        with open(p) as f:
            soup = BeautifulSoup(f, 'html.parser')
        
        for param in soup.find_all('param'):
            v = 'param -> name=' + param.get('name') + ' value=' + param.get('value')
            params.append((param.get('name'), param.get('value')))
            comm = 'rosparam set ' + param.get('name') + ' ' + param.get('value')
            param_process = Popen(
                shlex.split(comm)
            )
            listSubProcesses.append(param_process)

        paramFilePrefixes.clear()
        paramFiles.clear()
        for group in soup.find_all('group'):
            pack = group.get('ns')
            paramFilePrefixes.append(pack)
            for file in group.find_all('rosparam'):
                f = file.get('file')
                m = re.search('\$\(find (.+?)\)', f)
                if(m):
                    src = m.group(1)
                    f = f.replace('$(find ' + src + ')', relativePathToSrc + src)
                paramFiles.append((f, pack))
                v = 'rosparam load ' + f + ' /' + pack
                param_process = Popen(
                shlex.split(v)
                )
                listSubProcesses.append(param_process)

        msg = FoxgloveDataMessage()
        msg.command = "list_parameter_prefix"
        msg.data = paramFilePrefixes
        comm_data.publish(msg) 

        nodes.clear()
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
        
        # Get recent configs
        temp = {}
        with open(recentOpenedPath) as f:
            temp = json.load(f)

        res = {}
        files = []
        files.append({"name":configFile, "package": launchFiles[configFile]})
        count = 0
        for file in temp["files"]:
            if(file["name"] != configFile and file["package"] != "" and count < 2):
                files.append(file)
                count += 1
        res["files"] = files
        with open(recentOpenedPath, "w") as f:
            json.dump(res, f)
        

    # 2 -----------------------------------------------------

    # 3 -----------------------------------------------------

    # 4 -----------------------------------------------------

    if(data.command == "change_wind"):
        global speed
        global direction
        speed = data.data[0]
        direction = data.data[1]

    # 5 -----------------------------------------------------


def reccModif(list, param, value):
    if(not list):
        return value
    else:
        print(list)
        if(type(param)==dict):
            print("dict")
            print(list[0])
            param[list[0]] = reccModif(list[1:], param[list[0]], value)
        else:
            print("list")
            print(int(list[0]))
            param[int(list[0])] = reccModif(list[1:], param[int(list[0])], value)
        return param


"""Handles the modification of a parameter: pushes the changes to ROS and saves the parameter as modified

:param m message recieved from foxglove
:type m message type : {string parameter, string[] sub_param, string value}
"""
def modifyCallback(m):
    global modifiedParameters
    print("Modify recieved")
    param = rospy.get_param(m.parameter)
    list = m.sub_param
    value = m.value

    # Convert the booleans true and false to respectively True and False (python syntaxe)
    if(value == "true" or value == "false"):
        value = value.capitalize()
    
    # Convert the string to its value
    newParam = reccModif(list, param, ast.literal_eval(value))
    rospy.set_param(m.parameter, newParam)

    # Add the parameter to the modified list (to later save in file)
    file = m.parameter.split("/")
    if(file[1] in modifiedParameters):
        if(file[2] not in modifiedParameters[file[1]]):
            modifiedParameters[file[1]].append(file[2])
    else:
        modifiedParameters[file[1]] = [file[2]]


""" Creates the ROS publishers

"""
def caller():
    global launch_value
    # Publisher for test topic
    global test_pub
    test_pub = rospy.Publisher('tests', String, queue_size=10)

    # Publisher for commands topic
    global comm_pub
    comm_pub = rospy.Publisher('commands', String, queue_size=10)

    # Publisher for data passing
    global comm_data
    comm_data = rospy.Publisher('data', FoxgloveDataMessage, queue_size=10)

    global visualization_pub
    visualization_pub = rospy.Publisher('dataviz', Marker, queue_size=10)


""" Creates the ROS listeners

"""
def listener():
    
    rospy.init_node('foxglove_interface', anonymous=True)
    

    rospy.Subscriber("instructions", String, instruction_callback)
    rospy.Subscriber("data", FoxgloveDataMessage, dataCallBack)
    rospy.Subscriber("updates", Update, modifyCallback)

if __name__ == '__main__':
    try:
        caller()
    except rospy.ROSInterruptException:
        pass
    listener()
    
    state_pub = rospy.Publisher('simulation_state', FoxgloveDataMessage, queue_size=10)
    state_rate = rospy.Rate(10)
    MARKERS_MAX = 100
    count = 0
    markerArray = MarkerArray()
    rocket_pos = geometry_msgs.msg.Pose()
    marker = Marker()

    while not rospy.is_shutdown():
        msg = FoxgloveDataMessage()
        msg.command = str(launch_value.value)
        if(launch_value is SimulatorState.selection):
            msg.data = []
        if(launch_value is SimulatorState.configs):
            msg.data = [str(len(recentConfigs)), str(len(allConfigs)), *recentConfigs, *allConfigs]
        if(launch_value is SimulatorState.preLaunch):
            msg.data = [configFile, str(len(paramFilePrefixes)), *paramFilePrefixes]
        if(launch_value is SimulatorState.launching):
            msg.data = [configFile]
        if(launch_value is SimulatorState.launched):
            msg.data = [configFile, str(speed), str(direction)]
        if(launch_value is SimulatorState.simulation):
            msg.data = [configFile, str(speed), str(direction)]

        state_pub.publish(msg)

        if(launch_value is SimulatorState.preLaunch):
            v = 0
            # modelName = rospy.get_param("/visualization/stl_model")
            # rocket_scale = rospy.get_param("/visualization/stl_model_scale")
            # rocket_alpha = rospy.get_param("/visualization/stl_alpha")
            
            # marker.header.frame_id = "/rocket_model"
            # marker.ns = "rocket"
            # marker.type = marker.MESH_RESOURCE
            # marker.mesh_resource = modelName
            # marker.id = 0
            # marker.action = marker.ADD
            # marker.scale.x = rocket_scale
            # marker.scale.y = rocket_scale
            # marker.scale.z = rocket_scale
            # marker.color.a = rocket_alpha
            # marker.color.r = 0.75
            # marker.color.g = 0.75
            # marker.color.b = 0.75
            # marker.pose.orientation.w = 1.0
            # marker.pose.position.x = 0
            # marker.pose.position.y = 0
            # marker.pose.position.z = 0
            
                
            # visualization_pub.publish(marker)

        state_rate.sleep()
