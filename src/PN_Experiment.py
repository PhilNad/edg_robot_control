#!/usr/bin/env python

import sys
from os import popen, system, rename
import rospy
import time
import moveit_commander
import numpy as np
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

#Use the joint values, NOT MoveIt, to go to the absolute position
def goto_depth(depth):
    joint_values = []
    joint_values_0745= [-0.5136469046222132, -1.9015186468707483, 1.675382137298584, -1.3438003698932093, -1.5816224257098597, 1.917220950126648]
    joint_values_075 = [-0.503188435231344, -1.8894789854632776, 1.6585173606872559, -1.338740650807516, -1.5818622748004358, 1.9103862047195435]
    joint_values_076 = [-0.5032003561602991, -1.8855255285846155, 1.6379127502441406, -1.322146240864889, -1.5816825071917933, 1.9105420112609863]
    joint_values_077 = [-0.5033085981952112, -1.881392780934469, 1.6170082092285156, -1.305323902760641, -1.5816944281207483, 1.9107338190078735]
    joint_values_078 = [-0.5033205191241663, -1.8770087401019495, 1.5958638191223145, -1.2886694113360804, -1.5816944281207483, 1.9109375476837158]
    joint_values_079 = [-0.5075352827655237, -1.8737142721759241, 1.5754756927490234, -1.2721226851092737, -1.5815866629229944, 1.9074841737747192]

    if depth == 0.745:
        joint_values = joint_values_0745
    else:
        if depth == 0.75:
            joint_values = joint_values_075
        else:
            if depth == 0.76:
                joint_values = joint_values_076
            else:
                if depth == 0.77:
                    joint_values = joint_values_077
                else:
                    if depth == 0.78:
                        joint_values = joint_values_078
                    else:
                        if depth == 0.79:
                            joint_values = joint_values_079
                        else:
                            exit()
    #It seems that repeating the same command allow better precision
    for i in range(1,11):
        move_group.go(joint_values, wait=True)


def get_variance(M1_offset_pressure, M2_offset_pressure, M1_maximal_pressure, M2_maximal_pressure):
    command = "rostopic echo -n 200 /fingers/1/meanpressure | sed 's/data: //' | tr -d '-' | tr -s '\n'"
    response = str(popen(command).read())
    measures_array = response.split('\n')[:-1]
    i=0
    for m in measures_array:
        m_percent = float(int(m)-M1_offset_pressure) / float(M1_maximal_pressure-M1_offset_pressure)
        measures_array[i] = m_percent
        i += 1
    f1_var = np.var(measures_array)

    command = "rostopic echo -n 200 /fingers/2/meanpressure | sed 's/data: //' | tr -d '-' | tr -s '\n'"
    response = str(popen(command).read())
    measures_array = response.split('\n')[:-1]
    i=0
    for m in measures_array:
        m_percent = float(int(m)-M2_offset_pressure) / float(M2_maximal_pressure-M2_offset_pressure)
        measures_array[i] = m_percent
        i += 1
    f2_var = np.var(measures_array)

    return (f1_var, f2_var)

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
    depth_values    = [0.79]
    #Maximum protective closing position
    maximal_closing = 900
    #Goal pressure percentage values
    goal_pressure   = [0.2, 0.3, 0.4, 0.5, 0.6]

    approach_distance = 200 #Old: 400

    #Bias and maximum of the output of each transducer
    #It seems that this changes sometimes...better verify often their values
    #M1_offset_pressure  = 346 # Old: 285
    M1_maximal_pressure = 420 # Old: 420
    #M2_offset_pressure  = 365 # Old: 362
    M2_maximal_pressure = 420 # Old: 420

    for depth in depth_values:
        for percent in goal_pressure:
            goal_percent_M1 = percent
            goal_percent_M2 = percent

            #Get the pressure when the fingers are not touching anything.
            #As the offset changes from time to time, its better than hardcoding.
            (M1_offset_pressure, M2_offset_pressure) = fingersControl.get_mean_pressure()

            #Move to the initial Z position
            goto_depth(depth)

            filename = start_logging()

            #Close the fingers
            fingersControl  = twoFingersController(opening_speed=1.7, closing_speed=1.42, smallest_time=16, largest_gap=16)
            fingersControl.adjust_position(approach_distance, approach_distance)
            goal_pressure_M1 = goal_percent_M1*(M1_maximal_pressure-M1_offset_pressure) + M1_offset_pressure
            goal_pressure_M2 = goal_percent_M2*(M2_maximal_pressure-M2_offset_pressure) + M2_offset_pressure
            fingersControl.close_until_pressure(goal_pressure_M1, goal_pressure_M2, maximal_closing)

            #Allow a plateau to form and compute variance
            time.sleep(3)
            (f1_var, f2_var) = get_variance(M1_offset_pressure, M2_offset_pressure, M1_maximal_pressure, M2_maximal_pressure)
            print("F1 Variance: "+str(f1_var))
            print("F2 Variance: "+str(f2_var))


            #Old: Lift the pipe 6 centimeters up, 0.5 centimeter at a time in 12 steps
            #Old: Lift the pipe 4 centimeters up, 0.5 centimeter at a time in 8 steps
            f1_previous = 0
            f2_previous = 0
            for i in range(0,8):
                (f1_mean_pre, f2_mean_pre) = fingersControl.get_mean_percent_pressure(M1_offset_pressure, M2_offset_pressure, M1_maximal_pressure, M2_maximal_pressure)
                print("F1 Pressure: "+str(f1_mean_pre))
                print("F2 Pressure: "+str(f2_mean_pre))
                #The first time, we only record the percent pressure
                if i > 0:
                    #Compute the absolute slope
                    f1_slope = abs(f1_mean_pre-f1_previous)
                    f2_slope = abs(f2_mean_pre-f2_previous)
                    print("F1 Slope: "+str(f1_slope))
                    print("F2 Slope: "+str(f2_slope))
                    #Compute the minimum threshold slope
                    f1_m = -0.25*(f1_mean_pre**3)+0.475*(f1_mean_pre**2)-0.325*f1_mean_pre+0.1
                    f2_m = -0.25*(f2_mean_pre**3)+0.475*(f2_mean_pre**2)-0.325*f2_mean_pre+0.1
                    print("F1 Min: "+str(f1_m))
                    print("F2 Min: "+str(f2_m))
                    f1_threshold = max(2*f1_var, f1_m)
                    f2_threshold = max(2*f2_var, f2_m)
                    print("F1 Threshold: "+str(f1_threshold))
                    print("F2 Threshold: "+str(f2_threshold))
                    #Test the stopping condition
                    if (f1_slope > f1_threshold) and (f2_slope > f2_threshold):
                        print("Triggered stop condition.")
                        time.sleep(3)
                        break;
                f1_previous = f1_mean_pre
                f2_previous = f2_mean_pre
                #If the threshold is not reached, we pull further on the object
                robot.goRelPosition(goal_pos_rel=(0,0,0.005))

            #Return to original depth
            goto_depth(depth)

            #Open the fingers
            fingersControl  = twoFingersController(opening_speed=1.7, closing_speed=1.42, smallest_time=20, largest_gap=20)
            fingersControl.adjust_position(0, 0)

            stop_logging()
            new_filename = filename.replace(".csv","_"+str(depth)+"_"+str(goal_percent_M1)+"_"+str(goal_percent_M2)+".csv")
            rename(filename, new_filename)
            print("Data logged to "+new_filename)
