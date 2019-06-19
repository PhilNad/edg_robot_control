# This modules defines convenient functions used to
# accurately control the fingers

import sys
from os import popen, system

class twoFingersController:
    #Constructor speeds and timing are to be found experimentally and vary
    #with the mechanical setup used in the experiemnt.
    def __init__(self, opening_speed=1.7, closing_speed=1.42, smallest_time=20, largest_gap=10):
        self.opening_speed = opening_speed
        self.closing_speed = closing_speed
        self.smallest_time = smallest_time
        self.largest_gap   = largest_gap

    #This function creates a Linux command that sends a request to the service
    #which is responsible of sending commands to the fingers.
    def set_gripper_closing(self, motor1_position, motor2_position):
        print("Motor 1 position requested: "+str(motor1_position))
        print("Motor 2 position requested: "+str(motor2_position))
        command = "rosservice call /fingers/set_position "+str(int(motor1_position))+" "+str(int(motor2_position))
        system(command)

    #If you want to open the fingers for a number of encoder ticks,
    #this function tells you the command you should send to the speed controller
    #of the motor.
    def open_fingers(self, ticks):
        #Speed of the motor in ticks/millisecond when opening, this takers into
        #account the impact of the environment (eg. the spring)
        speed = self.opening_speed
        #milliseconds = ticks / speed
        ms = int(ticks/speed)

        #Smallest timelapse that actually moves the motors
        #This needs to be found experimentally and is influenced by the environment
        smallest_time = self.smallest_time
        #If the resulting command is too small, use the minimal command
        if ms < smallest_time:
            ms = smallest_time

        #If the result is even, we need to make it odd.
        #Thats how our protocol works.
        if (ms % 2) == 0:
            ms -= 1
        return ms

    #If you want to close the fingers for a number of encoder ticks,
    #this function tells you the command you should send to the speed controller
    #of the motor.
    def close_fingers(self, ticks):
        #Speed of the motor in ticks/millisecond when closing, this takers into
        #account the impact of the environment (eg. the spring)
        speed = self.closing_speed
        #milliseconds = ticks / speed
        ms = int(ticks/speed)

        #Smallest timelapse that actually moves the motors
        #This needs to be found experimentally and is influenced by the environment
        smallest_time = self.smallest_time
        #If the resulting command is too small, use the minimal command
        if ms < smallest_time:
            ms = smallest_time

        #If the result is odd, we need to make it even.
        #Thats how our protocol works.
        if (ms % 2) != 0:
            ms -= 1
        return ms

    #This function calculates finds the command that needs to be sent so
    #that the fingers RELATIVELY to our current position for a number of ticks
    #Positive number of ticks --> closes the fingers
    #Negative number of ticks --> opens the fingers
    def move_fingers(self, ticks):
        #A negative number of ticks means that we want to open the fingers
        #and a positive number of ticks means that we want to close them
        ms = 0
        if(ticks < 0):
            ms = self.open_fingers(abs(ticks))
        else:
            ms = self.close_fingers(abs(ticks))
        self.set_gripper_closing(ms,ms)

    #This function returns the current position of the fingers
    def get_fingers_position(self):
        m1_pos_cmd = "rostopic echo -n 1 /fingers/1/position | head -n 1 | sed 's/data: //'"
        m2_pos_cmd = "rostopic echo -n 1 /fingers/2/position | head -n 1 | sed 's/data: //'"

        current_m1_pos = int(popen(m1_pos_cmd).read())
        current_m2_pos = int(popen(m2_pos_cmd).read())

        return (current_m1_pos,current_m2_pos)


    #This function is meant to slowly adjust the position after an initial
    #approach move was done. Due to the large quantity of subsequent commands
    #that are sent through this function, its not meant to be used while logging
    #since it considerably reduce the bandwidth that can be used to receive data.
    def adjust_position(self,m1_goal_pos, m2_goal_pos):
        #Size of the acceptable difference to the goal position, in ticks.
        largest_gap = self.largest_gap
        #Number of ticks we want to move the fingers at every iteration.
        command_amplitude = self.smallest_time

        while True:
            command_m1 = 0
            command_m2 = 0

            #Get current position of the fingers
            (m1_pos, m2_pos) = self.get_fingers_position()
            print("Current M1: "+str(m1_pos))
            print("Current M2: "+str(m2_pos))

            #Difference between our goal position and our current
            delta_m1 = m1_goal_pos - m1_pos
            delta_m2 = m2_goal_pos - m2_pos

            print("Delta M1: "+str(delta_m1))
            print("Delta M2: "+str(delta_m2))

            #If its positive, it means we need to close the finger.
            #if its negative, we need to open it.
            if abs(delta_m1) > largest_gap:
                if delta_m1 > 0:
                    command_m1 = self.close_fingers(command_amplitude)
                else:
                    command_m1 = self.open_fingers(command_amplitude)

            if abs(delta_m2) > largest_gap:
                if delta_m2 > 0:
                    command_m2 = self.close_fingers(command_amplitude)
                else:
                    command_m2 = self.open_fingers(command_amplitude)

            #Move the fingers if need be
            if command_m1 > 0 or command_m2 > 0:
                self.set_gripper_closing(command_m1, command_m2)
            else:
                #If both commands are zero, it means we are done.
                return

    #Quick shortcut for the homing of the fingers
    def homing_fingers(self):
        self.adjust_position(0, 0)
        print("Homing sequence done.")
