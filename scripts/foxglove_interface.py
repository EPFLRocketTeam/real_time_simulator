#!/usr/bin/env python

# -----------------------
#
# Node to communicate with the foxglove interface
#
# -----------------------

from glob import glob
import rospy
import roslaunch
import rospkg
from yaml import load, dump
from catkin_pkg.topological_order import topological_order 
import time
from std_msgs.msg import String
from real_time_simulator.msg import Update
import shlex
from psutil import Popen


def tests(data):
    rospy.logdebug(data.data)

test_value = 0
launch_value = 0
global aList
aList = []
def instruction_callback(instruction):
    global launch_value
    global test_value
    print("Instruction recieved")
    print(instruction)
    if(instruction.data == "launch"):
        # TODO : Probably to replace with a state machine : {Idle, Launched, ...?}
        if(launch_value < 1):
            print("Launch")
            launch_value += 1
            command = String()
            comm_pub.publish(command)
            time.sleep(0.5)
    if(instruction.data == "reset"):
        print("reset")
    if(instruction.data == "stop"):
        print("stop")
    if(instruction.data == "get_packages"):
        res = topological_order("/home/mathieu/catkin_ws/src")
        
        print(res)
    if(instruction.data == "load_nodes"):
        print("loading nodes")
    if(instruction.data == "stop_node"):
        for p in aList:
            p.terminate()
        aList.clear()
    if(instruction.data == "launch_node"):
        #node = roslaunch.core.Node("rqt_gui","rqt_gui")
        #launch = roslaunch.scriptapi.ROSLaunch()
        #launch.start()
        #process = launch.launch(node)
        #print(process.is_alive)
        
        Popen(
            shlex.split('rosparam set /gnc_package "test_rocket_gnc"')
        )
        Popen(
            shlex.split('rosparam set /simulation 3')
        )
        Popen(
            shlex.split('rosparam load /home/mathieu/catkin_ws/src/test_rocket_gnc/config/rocket_parameters.yaml /rocket')
        )
        Popen(
            shlex.split('rosparam load /home/mathieu/catkin_ws/src/test_rocket_gnc/config/environment_parameters.yaml /environment')
        )
        Popen(
            shlex.split('rosparam load /home/mathieu/catkin_ws/src/real_time_simulator/config/perturbations_parameters.yaml /perturbation')
        )
        Popen(
            shlex.split('rosparam load /home/mathieu/catkin_ws/src/real_time_simulator/config/visualization_parameters.yaml /visualization')
        )
        node_process = Popen(
            shlex.split('rosrun test_rocket_gnc test_rocket_control')
        )
        aList.append(node_process)
        Popen(
            shlex.split('rosrun test_rocket_gnc test_rocket_navigation')
        )
        aList.append(node_process)
        node_process = Popen(
            shlex.split('rosrun test_rocket_gnc test_rocket_guidance')
        )
        aList.append(node_process)
        node_process = Popen(
            shlex.split('rosrun test_rocket_gnc test_rocket_fsm')
        )
        aList.append(node_process)
        node_process = Popen(
            shlex.split('rosrun test_rocket_gnc av_interface.py')
        )
        aList.append(node_process)
        node_process = Popen(
            shlex.split('rosrun real_time_simulator aerodynamic.py')
        )
        aList.append(node_process)
        node_process = Popen(
            shlex.split('rosrun real_time_simulator foxglove_interface.py')
        )
        aList.append(node_process)
        node_process = Popen(
            shlex.split('rosrun real_time_simulator disturbance.py')
        )
        aList.append(node_process)
        node_process = Popen(
            shlex.split('rosrun real_time_simulator integrator')
        )
        aList.append(node_process)
        node_process = Popen(
            shlex.split('rosrun real_time_simulator GUI_interface')
        )
        aList.append(node_process)
        node_process = Popen(
            shlex.split('rosrun rosbag record')
        )
        aList.append(node_process)

        
        
    if(instruction.data == "test"):
        test_value += 1
        msg = String()
        msg.data = "Message arrived" + str(test_value)
        test_pub.publish(msg)
        time.sleep(0.5)
    
def caller():
    # Publisher for test topic
    global test_pub
    test_pub = rospy.Publisher('tests', String, queue_size=10)

    # Publisher for commands topic
    global comm_pub
    comm_pub = rospy.Publisher('commands', String, queue_size=10)

def listener():
    
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('foxglove_interface', anonymous=True)

    rospy.Subscriber("instructions", String, instruction_callback)
    rospy.Subscriber("tests", String, tests)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    try:
        caller()
    except rospy.ROSInterruptException:
        pass
    listener()
