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
    fingersControl  = twoFingersController(opening_speed=1.7, closing_speed=1.42, smallest_time=20, largest_gap=20)

    #Depth values relative to the current depth
    depth_values    = [0.75, 0.76, 0.77, 0.78]
    #Maximum protective closing position
    maximal_closing = 900
    #Goal pressure percentage values
    goal_pressure   = [0.3, 0.4, 0.5, 0.6, 0.7]

    #High-level algorithm
    #1) Lower the arm to Z (4 different values)
    #2) Start logging
    #3) Close fingers  (each to one of 5 different values)
    #4) Stop logging
    #5) Go to #1

    #fingersControl  = twoFingersController(opening_speed=1.7, closing_speed=1.42, smallest_time=10, largest_gap=10)
    #fingersControl.adjust_position(0, 0)
    #exit()

    #Bias and maximum of the output of each transducer
    M1_offset_pressure  = 285
    M1_maximal_pressure = 420
    M2_offset_pressure  = 362
    M2_maximal_pressure = 420

    #For first test, overwrite the values, so we get 10 trials
    depth_values    = [0.76, 0.77]
    for depth in depth_values:
        for percent in goal_pressure:
            goal_percent_M1 = percent
            goal_percent_M2 = percent

            #Move to the initial Z position
            current_pose = move_group.get_current_pose().pose
            current_pose.position.z = depth
            robot.goPose(current_pose)

            filename = start_logging()

            #Close the fingers
            fingersControl  = twoFingersController(opening_speed=1.7, closing_speed=1.42, smallest_time=16, largest_gap=16)
            fingersControl.adjust_position(400, 400)
            goal_pressure_M1 = goal_percent_M1*(M1_maximal_pressure-M1_offset_pressure) + M1_offset_pressure
            goal_pressure_M2 = goal_percent_M2*(M2_maximal_pressure-M2_offset_pressure) + M2_offset_pressure
            fingersControl.close_until_pressure(goal_pressure_M1, goal_pressure_M2, 900)

            #Lift the pipe 4 centimeters up, 0.5 centimeter at a time in 8 steps
            move_group.set_max_velocity_scaling_factor(0.1)
            move_group.set_max_acceleration_scaling_factor(0.1)
            
            initial_pose = move_group.get_current_pose().pose
            for i in range(1,9):
                current_pose = move_group.get_current_pose().pose
                current_pose.position.z = initial_pose.position.z + i*0.005
                robot.goPose(current_pose)

            #Return to original depth
            current_pose = move_group.get_current_pose().pose
            current_pose.position.z = depth
            robot.goPose(current_pose)

            move_group.set_max_velocity_scaling_factor(1)
            move_group.set_max_acceleration_scaling_factor(1)

            #Open the fingers
            fingersControl  = twoFingersController(opening_speed=1.7, closing_speed=1.42, smallest_time=10, largest_gap=10)
            fingersControl.adjust_position(0, 0)

            stop_logging()
            new_filename = filename.replace(".csv","_"+str(depth)+"_"+str(goal_percent_M1)+"_"+str(goal_percent_M2)+".csv")
            rename(filename, new_filename)
            print("Data logged to "+new_filename)
