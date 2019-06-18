#This modules manages the movements of the simulated robot
#through MoveIt and the Gripper Service.

from os import popen, system
from pose_utils import poseUtilities
from math import pi, radians
from std_msgs.msg import UInt8

class robControl:
    def __init__(self, rospy_instance, move_group):
        self.rospy          = rospy_instance
        self.move_group     = move_group
        self.ur10_joints    = {'base':0,'shoulder':1,'elbow':2,'wrist_1':3,'wrist_2':4,'wrist_3':5}

        #These offset should be measured and set as precisely as possible
        #The distance between flange and fingertip is actually 0.2 but it seems that the URDF model is shorter and thus
        #and additionnal 0.09 is added to get the desired real-world behavior.
        self.poseUtils      = poseUtilities(self.rospy,self.move_group, WORLD_TO_BASE=(0,0,0), FLANGE_TO_FINGERTIP=0.04)

    def printCurrentPose(self):
        self.poseUtils.printCurrentPose()

    #Stops any residual movement from MoveIt
    def stop(self):
        self.move_group.stop()
        self.move_group.clear_pose_targets()


    def set_gripper_closing(self, motor1_position, motor2_position):
        print("Motor 1 position requested: "+str(motor1_position))
        print("Motor 2 position requested: "+str(motor2_position))
        command = "rosservice call /fingers/set_position "+str(int(motor1_position))+" "+str(int(motor1_position))
        system(command)

    #Command the gripper's opening relative to current opening
    def set_relative_gripper_closing(self, delta_motor1_position=0,  delta_motor2_position=0):
        #Get current position from the appropriate ROS topics
        #This does not involve subscribing to a topic...
        #Dont ask for a stream if you only need a single drop
        m1_pos_cmd = "rostopic echo -n 1 /fingers/1/position | head -n 1 | sed 's/data: //'"
        m2_pos_cmd = "rostopic echo -n 1 /fingers/2/position | head -n 1 | sed 's/data: //'"

        current_m1_pos = int(popen(m1_pos_cmd).read())
        current_m2_pos = int(popen(m2_pos_cmd).read())

        #Add the relative position to current's
        goal_m1_pos = current_m1_pos + delta_motor1_position
        goal_m2_pos = current_m2_pos + delta_motor2_position

        #Set the corresponding position
        self.set_gripper_closing(goal_m1_pos, goal_m2_pos)

    #Moves the orientation of the wrist relative to its current position.
    def goRelOrientation(self, wrist_1=0, wrist_2=0, wrist_3=0):
        joint_goal = self.move_group.get_current_joint_values()
        joint_goal[self.ur10_joints['wrist_1']]     += radians(wrist_1)
        joint_goal[self.ur10_joints['wrist_2']]     += radians(wrist_2)
        joint_goal[self.ur10_joints['wrist_3']]     += radians(wrist_3)
        res = self.move_group.go(joint_goal, wait=True)
        print('Execution returned:'+str(res))

    #Move the end effector relative to its current pose
    def goRelPosition(self,goal_pos_rel=(0,0,0), sync=True):
        #This is added to protect the user against dangerous unit errors.
        if(goal_pos_rel[0]+goal_pos_rel[1]+goal_pos_rel[2] > 2):
            print("Watch out! Relative movement bigger than 2 meters. Aborting.")
            return
        #For implementation details, see self.go()
        pose_goal = self.poseUtils.makeIncrementalEEPose(position=goal_pos_rel, orientation=(0,0,0))
        #Go to the pose
        self.goPose(pose_goal, sync=sync)

    #Go to the specified pose
    def goPose(self, goal_pose, sync=True):
        self.move_group.set_pose_target(goal_pose)

        #Return a motion plan (a RobotTrajectory) to the set goal state
        plan = self.move_group.plan()

        #Execute the plan if one was found
        res = self.move_group.execute(plan,wait=sync)
        print('Execution returned:'+str(res))
