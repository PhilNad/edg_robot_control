#!/usr/bin/env python

import sys
from os import popen, system
import rospy
import moveit_commander
from two_fingers_control import twoFingersController

def INCHES_TO_MM(measure_in_inches):
    return 25.4*measure_in_inches

def MM_TO_M(measure_in_millimeters):
    return 0.001*measure_in_millimeters

def INCHES_TO_M(measure_in_inches):
    return 0.0254*measure_in_inches

def start_logging():
    command = "rosservice call /data_logging 1 | sed 's/OutputFileName: \"//' | sed 's/\"//'"
    filename = str(popen(command).read())
    return filename

def stop_logging():
    command = "rosservice call /data_logging 0 | sed 's/OutputFileName: \"//' | sed 's/\"//'"
    filename = str(popen(command).read())
    return filename


if __name__ == "__main__":
    #Initialization
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('edg_test', anonymous=True)
    
    fingersControl = twoFingersController(opening_speed=1.7, closing_speed=1.42, smallest_time=20, largest_gap=10)

    filename = start_logging()
    print("Logging to "+filename)
    fingersControl.move_fingers(200)
    fingersControl.adjust_position(200,200)
    rospy.sleep(0.1)

    fingersControl.move_fingers(-100)
    fingersControl.adjust_position(100,100)
    rospy.sleep(0.1)

    fingersControl.move_fingers(-100)
    fingersControl.adjust_position(0,0)
    rospy.sleep(0.1)

    stop_logging()
