#!/usr/bin/env python

# -----------------------
#
# Node to communicate with the foxglove interface
#
# -----------------------

import rospy
from os import listdir, rename, kill, remove
from os.path import isfile, join, isdir,exists
import json
from datetime import datetime
from catkin_pkg.topological_order import topological_order 
import time
from std_msgs.msg import String
from real_time_simulator.msg import Update, Data
import shlex
from psutil import Popen
from bs4 import BeautifulSoup
import re
import signal
from enum import Enum
import ruamel.yaml

import dynamic_reconfigure.client
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


# Stop nodes (kill all nodes)
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


# Launches ros nodes
def launch_nodes():
    global launch_value
    launch_value = SimulatorState.launching

    for node in nodes:
        v = 'rosrun ' + node[1] + ' ' + node[2] + ' ' + node[3] + ' __name:=' + node[0]
        print(v)
        node_process = Popen(
            shlex.split(v)
        )
        listSubProcesses.append(node_process)

    # Set default values for wind speed and wind direction
    global speed
    global direction
    global client
    client = dynamic_reconfigure.client.Client("aerodynamic", timeout=30, config_callback=conf_callback)
    env_data = rospy.get_param("/environment")
    speed = env_data["wind_speed"]
    direction = env_data["wind_direction"]
    client.update_configuration({"wind_speed" : speed, "wind_direction": direction})
    launch_value = SimulatorState.launched


# Clears parameters from ros system
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


# Get all configs
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
    msg = Data()
    msg.command = "recent_configs"
    msg.data = res
    comm_data.publish(msg) 

    #Get all config files
    onlyfiles = []

    res = topological_order(relativePathToSrc)
    for elem in res:
        path = relativePathToSrc + elem[0] + "/launch/"
        if(isdir(path)):
            temp = [f for f in listdir(path) if isfile(join(path, f)) and "rocket_" in f]
            onlyfiles = onlyfiles + temp
            for file in temp:
                launchFiles[file] = elem[0]

    allConfigs = onlyfiles
    # Give back the results
    msg = Data()
    msg.command = "configs"
    msg.data = onlyfiles
    comm_data.publish(msg)



# Callback for the instruction topic
def instruction_callback(instruction):
    global launch_value
    global test_value
    global launchFiles
    print("Instruction recieved")
    # 1 -----------------------------------------------------

    # Instruction to get all config that can be launched
    if(instruction.data == "get_configs"):
        # Get all config files
        print("Get config ------------")
        getConfigs()
        launch_value = SimulatorState.configs

    # 2 -----------------------------------------------------

    # Instruction to stop the simulation
    if(instruction.data == "clear_parameters"):
        print("Clear parameters ----------")
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
                print("Yaml file ---------")
                print(par)
                changed = modifiedParameters[file[1]]
                
                for p in changed:
                    par[p] = rospy.get_param(file[1] + "/" + p)
                f = open(file[0], "w")
                yaml.dump(par, f)
                f.close()
                print(modifiedParameters[file[1]])
        modifiedParameters.clear()
    # 3 -----------------------------------------------------

    # Close config instruction
    if(instruction.data == "close_config"):
        print("Stop config " + configFile)
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
        print("Pressed launch, state = " + str(launch_value) + " =? " + str(SimulatorState.launched) + " -> " + str(launch_value is SimulatorState.launched))
        if(launch_value is SimulatorState.launched):
            print("Launch")
            launch_value = SimulatorState.simulation
            command = String()
            comm_pub.publish(command)
            time.sleep(0.5)

    # Instuction to stop nodes
    if(instruction.data == "stop_nodes"):
        print("Stopping nodes --------")
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
        print("Stopping nodes --------")
        stop_simulation()
    
    # Instruction to reset the simulation
    if(instruction.data == "restart_simulation"):
        print("Reset simulation ---------")
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
        print(p)
        
        with open(p, "w") as f:
            f.write(str(soup.prettify()))
    
    print("Current state -> " + str(launch_value))
    
    
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
    global launch_value
    global configFile
    print("data recieved--------------")
    print(data)

    # 1 -----------------------------------------------------

    # Select the configuration
    if(data.command == "select_config"):
        print("Setting config")
        launch_value = SimulatorState.preLaunch
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
            listSubProcesses.append(param_process)

        print("Extract param files :")
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
                    print(src)
                paramFiles.append((f, pack))
                v = 'rosparam load ' + f + ' /' + pack
                print(v)
                param_process = Popen(
                shlex.split(v)
                )
                listSubProcesses.append(param_process)

        msg = Data()
        msg.command = "list_parameter_prefix"
        msg.data = paramFilePrefixes
        comm_data.publish(msg) 

        print("Extract nodes :")
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

    # Update a parameter
    if(data.command == "update_param"):
        comm = 'rosparam set /environment/apogee "[0, 0, 2000]"'
        print(comm)
        param_process = Popen(
            shlex.split(comm)
        )
        listSubProcesses.append(param_process)

    # 3 -----------------------------------------------------

    # 4 -----------------------------------------------------

    if(data.command == "change_wind"):
        global client
        global speed
        global direction
        speed = data.data[0]
        direction = data.data[1]
        print("direction " + direction + " speed:" + speed)
        client.update_configuration({"wind_speed" : speed, "wind_direction": direction})

    # 5 -----------------------------------------------------


    print("Current state -> " + str(launch_value))

def modifyCallback(m):
    global modifiedParameters
    print("Modify recieved")
    comm = 'rosparam set /' + m.config + '/' + m.parameter + ' ' + m.value
    Popen(
        shlex.split(comm)
    )

    if(m.config in modifiedParameters):
        modifiedParameters[m.config].append(m.parameter)
    else:
        modifiedParameters[m.config] = [m.parameter]
    print(m)


# Initialises all publishers
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
    comm_data = rospy.Publisher('data', Data, queue_size=10)

def conf_callback(config):
    print("Conf callback : ")
    print(config)

# Initialises all listened topics and give their callback
def listener():
    
    rospy.init_node('foxglove_interface', anonymous=True)
    

    rospy.Subscriber("instructions", String, instruction_callback)
    rospy.Subscriber("data", Data, dataCallBack)
    rospy.Subscriber("updates", Update, modifyCallback)

if __name__ == '__main__':
    try:
        caller()
    except rospy.ROSInterruptException:
        pass
    listener()
    
    state_pub = rospy.Publisher('simulation_state', Data, queue_size=10)
    state_rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        msg = Data()
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

        state_rate.sleep()
