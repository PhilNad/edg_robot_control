#!/usr/bin/env python

import sys
from os import popen, system, rename
import rospy
import moveit_commander
from geometry_msgs.msg import Pose

#edg_robot_control's modules
from two_fingers_control import twoFingersController
from scene_manager import sceneManager
from robot_control import robControl

def INCHES_TO_MM(measure_in_inches):
    return 25.4*measure_in_inches

def MM_TO_M(measure_in_millimeters):
    return 0.001*measure_in_millimeters

def INCHES_TO_M(measure_in_inches):
    return 0.0254*measure_in_inches

def start_logging():
    command = "rosservice call /data_logging 1 | sed 's/OutputFileName: \"//' | sed 's/\"//'"
    filename = str(popen(command).read())[:-1]
    return filename

def stop_logging():
    command = "rosservice call /data_logging 0 | sed 's/OutputFileName: \"//' | sed 's/\"//'"
    filename = str(popen(command).read())[:-1]
    return filename

def quick_notes():
    '''
    Initial manual procedure:
        1) Manually move the robot right above the 4-screws attach.
        2) Screw the four screws so the robot holds the gripper
        3) Remove the perpendicular resting alu. extrusions.
        4) Close the fingers so the second phalanx barely touch the pipe
        5) Take a note of the current height (Z axis about 0.7283) and the position
            of the fingers (should be about 350)
        6) Manually move the robot so the covered barbs are just a bit above the
            water and the air begins to come in. Note the Z axis position at
            that point, should be around 0.7786
        7) Make sure the depth_values chosen in this program will respect the
            previously found Z axis limits.
        8) Make sure the closing values chosen in this program are relatively
            sound.

    This pose is the lower we can get before the palm start touching the pipe:

    This pose is the highest we can get before too much air enter through the barbs
    but it required the fingers to be partially closed:

    Exp #1: Initial depth = 0.7310 , Closure (350, 345)

    '''
    return


if __name__ == "__main__":
    #Initialization
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('edg_pn_experiment', anonymous=True)
    #The "manipulator" planning group as described in the SRDF represents the UR10
    move_group  = moveit_commander.MoveGroupCommander("manipulator")
    #Manages robot's movements
    robot           = robControl(rospy, move_group)
    #Manages fingers movements
    fingersControl  = twoFingersController(opening_speed=1.7, closing_speed=1.42, smallest_time=20, largest_gap=10)

    #Depth values relative to the current depth
    depth_values    = [0, 0.01, 0.02, 0.03]
    #Each depth has its set of closing values
    closing_values  = [ [400, 500, 600, 700, 800],
                        [425, 525, 625, 725, 825],
                        [450, 550, 650, 750, 850],
                        [475, 575, 675, 775, 875],
                        [500, 600, 700, 700, 900] ]

    #High-level algorithm
    #1) Lower the arm to Z (4 different values)
    #2) Start logging
    #3) Close fingers  (each to one of 5 different values)
    #4) Stop logging
    #5) Go to #1

    #Move to the minimal closing value in our list so the fingertips are deeper
    #in the water and that gives us more room in terms of depth so the air dont
    #enter the tubes.
    fingersControl.adjust_position(min(min(closing_values)), min(min(closing_values)))

    counter = 0
    previous_depth = 0
    for depth in depth_values:

        #Dont forget that this command is RELATIVE
        robot.goRelPosition(goal_pos_rel=(0,0,depth-previous_depth))

        this_depth_closing_values = closing_values[counter]
        for finger1_closing in this_depth_closing_values:
            for finger2_closing in this_depth_closing_values:

                filename = start_logging()

                #Move to the goal position
                fingersControl.adjust_position(finger1_closing, finger2_closing)

                stop_logging()

                new_filename = filename.replace(".csv","_"+str(depth)+"_"+str(finger1_closing)+"_"+str(finger2_closing)+".csv")
                rename(filename, new_filename)
                print("Data logged to "+new_filename)
        fingersControl.adjust_position(min(min(closing_values)), min(min(closing_values)))
        counter += 1
