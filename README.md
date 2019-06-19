# EDG Robot Control ROS Package (Node/Python)

## Objective
The objective of this package is to regroup in one place every modules useful for the control of EDG's UR-10 robotic arm. This package also expose a launch file that is useful to quickstart all programs related to the control of the arm.

## How to use the launch file?
The included launch file takes care of loading the appropriate robot description, the parameters and the configuration of the UR-10 arm through the usage of the [universal_robot](https://github.com/ros-industrial/universal_robot) ROS package. Furthermore, this launch file spawn an instance of [MoveIt](https://docs.ros.org/kinetic/api/moveit_tutorials/html/index.html), again using the files coming with the [universal_robot](https://github.com/ros-industrial/universal_robot) package. Finally, this launch file also spawn an instance of the [rviz](http://wiki.ros.org/rviz/UserGuide) program which let's the user see a virtual robot with its environment as well as allowing to interact with the robot.

This launch file takes a single argument which is the IP address of the robot. By default, a direct connection is assumed and the IP address of 10.0.0.1 is assumed to be the one of the robot. As all launch files, the usage is pretty simple:
```bash
roslaunch edg_robot_control edg_robot_control.launch
```

## How does it work?
The principal script uses three different [Python modules](https://docs.python.org/2/tutorial/modules.html) in order to work. Each of these modules have a specific purpose.

### The robot_control module
The objective of this module is to send commands to the robot such that it moves as intended. To do so, three main functions can be used: goRelOrientation, goRelPosition and goPose. The first is used to move the wrist of the robot by a number of degrees. The second is used to translate the robot end effector in space relative to its current position and following the world reference frame. The third function is used to command the robot to go to a specific precomputed pose.

Although it would be entirely possible to calculate the destination pose relative to the world reference frame and to use exclusively the goPose function, it would also make things more complicated than what is required for most common tasks. An easier workflow would be:
1) Use the teach pendant to reach the desired position.
2) Run the PrintCurrentPose.py script in order to generate the lines of code needed to reach that position/orientation.
3) In your program, use the previously generated lines of code to instruct the robot to reach that chosen pose.
4) Then translate the end effector using the goRelPosition function.
5) Then, if needed, change the orientation of the end effector using the goRelOrientation function.
6) Rinse and repeat the last two steps as needed.

### The scene_manager module
This module is used to manage the virtual scene in which MoveIt controls the robot. This scene defines objects in which the robot should not collide. Therefore it defines additional contraints that MoveIt should respect while trying to solve the optimisation problem that is path planning. For the moment, all objects are made of a number of boxes although other geometric primitives could be used.

### The pose_utils module
This module's primary purpose is to define helper functions to generate or transform poses. 
