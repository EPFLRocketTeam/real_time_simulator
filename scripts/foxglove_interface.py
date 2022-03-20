#!/usr/bin/env python

# -----------------------
#
# Node to communicate with the foxglove interface
#
# -----------------------

import rospy
import time
from std_msgs.msg import String
from real_time_simulator.msg import Update


def tests(data):
    rospy.logdebug(data.data)

test_value = 0
launch_value = 0
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
    if(instruction.data == "load_nodes"):
        print("loading nodes")
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
