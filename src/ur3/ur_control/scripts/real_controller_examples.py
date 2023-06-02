#!/usr/bin/env python
# -*- coding: utf-8 -*-

# ==============================================================================#
# real_controller_examples.py                                                   #
# ==============================================================================#
#                                                                               #
# DESCRIPTION:                                                                  #
# This script controls the movements of a UR robotic arm, making use of the     #
# ROS (Robot Operating System) environment. It contains functionalities for     #
# moving the arm to predefined joint positions, controlling the end-effector,   #
# and managing the digital output state related to the robotic gripper.         #
# The code does not only enable movements, but callibration and presentation    #
# routines as well                                                              #
#                                                                               #
#                                                                               #
# AUTHORS:                                                                      #
# - cambel (source code, drivers and overall ROS workspace)                     #
# - susi1603                                                                    #
# - riaj0224                                                                    #
#                                                                               #
# DATE : June 1st, 2023                                                         #
#                                                                               #
# VERSION: 1.0                                                                  #
#                                                                               #
# REQUIREMENTS:                                                                 #
# This script requires Python 2.x and the following Python libraries:           #
# - ur_control                                                                  #
# - argparse                                                                    #
# - rospy                                                                       #
# - timeit                                                                      #
# - numpy                                                                       #
# - tf                                                                          #
#                                                                               #
# NOTES:                                                                        #
# Please refer to the documentation to see full instalation and integration     #
#                                                                               #
# ==============================================================================#




#===============================================================================#
# SECTION: Imports                                                              #
# DESCRIPTION: These libraries provide a diverse set of tools for building a    # 
#              Python-based control system for a UR robotic arm, likely in a    #
#              ROS (Robot Operating System) environment.                        #
# INPUTS: N/A                                                                   #
# OUTPUTS: N/A                                                                  #
#===============================================================================#

from ur_control import transformations                                          # Customlibrary related to controlling a UR (Universal 
                                                                                # Robots) robot: library which handles various mathematical
                                                                                # transformations, like positioning and orienting the robot 
                                                                                # in 3D space.
                                                                                
from ur_control.arm import Arm                                                  # Library that specifically deals with the control and 
                                                                                # operation of a robotic arm.
                                                                                 
import argparse                                                                 # This is a standard Python library used for writing 
                                                                                # user-friendly command-line interfaces. It parses 
                                                                                # arguments and options from sys.argv, which are the 
                                                                                # command-line arguments passed to a Python script.

import rospy                                                                    # Python library for ROS, used for interfacing with the 
                                                                                # ROS system                                 

import timeit                                                                   # provides tools for timing Python code execution, which 
                                                                                # can be important for performance tuning and optimization
                                                                                #  in real-time systems like robotics. 

import numpy as np                                                              #  library for numerical computation in Python. It is used 
                                                                                # heavily in scientific computing and likely powers a lot 
                                                                                # of the mathematical operations in the system, such as 
                                                                                # the transformations for the UR robot.

import tf                                                                       # Manages the relationships between different coordinate 
                                                                                # frames over time, which is crucial for keeping track of 
                                                                                # the robot's position and orientation in 3D space.

np.set_printoptions(suppress=True)
np.set_printoptions(linewidth=np.inf)

from ur_msgs.srv import *                                                       # Provides ROS service definitions specific to UR robots, 
                                                                                # facilitating communication between different parts of the
                                                                                #  ROS system

import sys                                                                      # Provides tools for interacting with the Python runtime 
                                                                                # environment, which can be used for various purposes 
                                                                                # such as managing program flow and handling 
                                                                                # system-specific parameters.

#===============================================================================#
# CLASS: DigitalOutputManager                                                   #
# DESCRIPTION: This class manages digital output for a UR robotic arm. It       #
#              sets initial pin states, allows toggling of digital output, and  #
#              restarts the toggling process.                                   #
# INPUTS: initial_states - initial pins                                         #
# OUTPUTS: N/A                                                                  #
#===============================================================================#

class DigitalOutputManager:

    #===============================================================================#
    # FUNCTION: __init__                                                            #
    # DESCRIPTION: Initializes the DigitalOutputManager object by setting the       #
    #              initial states of the pins, making service calls to set the      #
    #              digital outputs to the initial states, and send a step train     #
    # INPUTS: initial_states - initial pin states                                   #
    # OUTPUTS: N/A                                                                  #
    #===============================================================================#
    def __init__(self, initial_states):

        self.pin_states = initial_states                                            # Array[3]: enable, direction, steps
        self.pins = [0, 1, 2]                                                       # Array[3]: Selected pins (digital outputs)                            
        self.toggle_count = 0                                                       # Software enabler for toggling 
        self.rate = rospy.Rate(5000)                                                # Frequency for step train: 5000 Hz -> 1 edge
                                                                                    # Period: 200ms x 2 (rising + falling edge) = 400ms
                                                                                    
        # First it is necessary to reflect the pins' initial states into the hardware,
        # and since the step train can wait, we first set the enable and direction pins
        for i in range(2):
            rospy.wait_for_service('/ur_hardware_interface/set_io')                 # Search for service                                  
        try:
            set_io = rospy.ServiceProxy('/ur_hardware_interface/set_io', SetIO)     # Stablish connection and define message type             
            resp = set_io(fun=1, pin=i, state=self.pin_states[i])                   # Set values
            # print(resp.success)                                                
        except rospy.ServiceException as e:                                         # In case of error showcase error message
            print("Service call failed: %s" % e) 

        # Next, just to ensure a good connection we connect again to the service to send 
        # the step train
        rospy.wait_for_service('/ur_hardware_interface/set_io')                     # Search for service

        try:
            set_io = rospy.ServiceProxy('/ur_hardware_interface/set_io', SetIO)     # Stablish connection and define message type

            if (self.toggle_count == 0):                                            # Condition for software enabler

                for i in range(800):                                                # Number of edges to send
                    resp = set_io(fun=1, pin=2, state=self.pin_states[2])           # Set new values to generate step train

                    # rospy.sleep(0.0005)                                           # Alternative for next line, not that precise
                    self.rate.sleep()                                               # Sleep for stablished period
                                                                                    
                                                                                    # Toggle step value
                    if self.pin_states[2] == 1.0:
                        self.pin_states[2] = 0.0
                    elif self.pin_states[2] == 0.0:
                        self.pin_states[2] = 1.0 

                self.toggle_count=1                                                 # Stop software enabler 

        except rospy.ServiceException as e:                                         # In case of error
            print("Service call failed: %s" % e)                                    # Showcase error


    #===============================================================================#
    # FUNCTION: toggle_digital_output                                               #
    # DESCRIPTION: Toggles the digital output by making a service call to change the#
    #              digital output state of pin 2 n-times.                           #
    # INPUTS: N/A                                                                   #
    # OUTPUTS: N/A                                                                  #
    #===============================================================================#
    def toggle_digital_output(self):

        rospy.wait_for_service('/ur_hardware_interface/set_io')                     # Search for service

        try:
            set_io = rospy.ServiceProxy('/ur_hardware_interface/set_io', SetIO)     # Stablish connection and define message type

            if (self.toggle_count == 0):                                            # Condition for software enabler

                for i in range(800):                                                # Number of edges to send
                    resp = set_io(fun=1, pin=2, state=self.pin_states[2])           # Set new values to generate step train

                    # rospy.sleep(0.0005)                                           # Alternative for next line, not that precise
                    self.rate.sleep()                                               # Sleep for stablished period
                                                                                    
                                                                                    # Toggle step value
                    if self.pin_states[2] == 1.0:
                        self.pin_states[2] = 0.0
                    elif self.pin_states[2] == 0.0:
                        self.pin_states[2] = 1.0 

                self.toggle_count=1                                                 # Stop enabler 

        except rospy.ServiceException as e:                                         # In case of error
            print("Service call failed: %s" % e)                                    # Showcase error
   

    #===============================================================================#
    # FUNCTION: restart_toggling                                                    #
    # DESCRIPTION: Resets the toggling process by setting the toggle_count to 0 and #
    #              changing the direction of the gripper.                           #
    # INPUTS: N/A                                                                   #
    # OUTPUTS: N/A                                                                  #
    #===============================================================================#
    def restart_toggling(self):
        self.toggle_count = 0                                                       # Re-enable software enabler

                                                                                    # Change direction of the gripper
        if self.pin_states[1] == 1.0:
            self.pin_states[1] = 0.0
        elif self.pin_states[1] == 0.0:
            self.pin_states[1] = 1.0                                                

        rospy.wait_for_service('/ur_hardware_interface/set_io')                     # Search for service                                  
        try:
            set_io = rospy.ServiceProxy('/ur_hardware_interface/set_io', SetIO)     # Stablish connection and define message type                      
            resp = set_io(fun=1, pin=1, state=self.pin_states[1])                   # Set values
            # print(resp.success)                                                
        except rospy.ServiceException as e:                                         # In case of error showcase error message
            print("Service call failed: %s" % e) 

        self.toggle_digital_output()                                                # Excecute toggle function


#===============================================================================#
# FUNCTION: move_joints                                                         #
# DESCRIPTION: This function commands the robot arm to move to a specific set of#
#              joint positions. After the move, it initializes the              #
#              DigitalOutputManager, likely controlling a gripper.              #
# INPUTS: wait (optional) - If true, the function will wait for the arm to      #
#                           reach the position before proceeding. Default is    #
#                           True                                                #
# OUTPUTS: N/A                                                                  #
#===============================================================================#
def move_joints(wait=True):
    q = [-0.0916, -1.971, 2.187, -3.358, -1.626, 0.176]                         # 'q' holds the desired joint positions for the robot 
                                                                                # arm in radians
    
    arm.set_joint_positions(position=q, wait=True, t=1.0)                       # Command the robot arm to move to the joint positions 
                                                                                # defined in 'q'.The robot is set to move to the joint
                                                                                #  positions within a time frame of 0.5 seconds
                                                                                # If 'wait' is true, it will block the execution 
                                                                                # until the robot has moved to the desired positions

    rospy.sleep(2.0)                                                            # Pause execution for 2 seconds


#===============================================================================#
# FUNCTION: move_joints2                                                        #
# DESCRIPTION: This function commands the robot arm to move to a specific set of#
#              joint positions. After the move, it initializes the              #
#              DigitalOutputManager, likely controlling a gripper.              #
# INPUTS: wait (optional) - If true, the function will wait for the arm to      #
#                           reach the position before proceeding. Default is    #
#                           True                                                #
#         position: position selector                                           #
# OUTPUTS: N/A                                                                  #
#===============================================================================#
def move_joints2(position,wait=True):
    
    if position == 1:
        q = [0.557, -1.131, 2.021, -4.009,-1.626 , 0.176]                       # 'q' holds the desired joint positions for the robot 
                                                                                # arm in radians
    if position == 2:
       q = [-0.069, -1.678, 1.747, -3.226,-1.621 , 0.168]                       # 'q' holds the desired joint positions for the robot 
                                                                                # arm in radians
    
    arm.set_joint_positions(position=q, wait=True, t=1.0)                       # Command the robot arm to move to the joint positions 
                                                                                # defined in 'q'.The robot is set to move to the joint
                                                                                #  positions within a time frame of 0.5 seconds
                                                                                # If 'wait' is true, it will block the execution 
                                                                                # until the robot has moved to the desired positions

    rospy.sleep(2.0)                                                            # Pause execution for 2 seconds

#===============================================================================#
# FUNCTION: move_endeffector                                                    #
# DESCRIPTION: This function aims to move the robot arm's end-effector based on #
#              specific transformations. It corrects the position based on      #
#              camera and gripper offsets, and also allows for control of a     #
#              gripper using DigitalOutputManager.                              #      
# INPUTS: wait (optional) - If true, the function will wait for the arm to reach#
#                           the position before proceeding. Default is True.    #
# OUTPUTS: N/A                                                                  #
#===============================================================================#
def move_endeffector(wait=True):
    
    # Set initial values for the end-effector of the arm to reach the desired
    # position
    cpose = arm.end_effector()                                                  # Get the current position of the end effector

    listener = tf.TransformListener()                                           # Instantiate a TransformListener object which
                                                                                # is used to receive transform information over 
                                                                                # time

    rate = rospy.Rate(10.0)                                                     # Set the rate of loop execution

                                                                                # Define various correction offsets
    d_cam_corr_x = -0.0678                                                     
    d_cam_corr_y = -0.0254

    corr_gripper_y = 0.145
    corr_gripper_x = -0.09
    corr_gripper_z = 0.07

    # Define the transformation matrices and search for the transform between 
    # the ArUCo and the base
    while not rospy.is_shutdown():                                              # Main loop runs until the ROS node is shut down
                                                                                # while continously searching for the transform

        try:
            (transAruco,rotAruco) = listener.lookupTransform('/base_link', '/aruco_marker_frame', rospy.Time(0)) 
                                                                                # Get the transform 
                                                                                # between '/base_link' 
                                                                                # and '/aruco_marker_frame' 
                                                                                # and '/wrist_3_link' frames

            #(transEE,rotEE) = listener.lookupTransform('/base_link', '/wrist_3_link', rospy.Time(0))
            
            Zpose = arm.end_effector()                                          # Get the current end effector position

            print(transAruco)                                                   # Print the position in space to which 
                                                                                # the robot is going to move

            x1 = transAruco[0]+d_cam_corr_x+corr_gripper_x
            dx1 = dynamic_x_offset(x1)

            y1 = transAruco[1]+d_cam_corr_y+corr_gripper_y
            z1 = transAruco[2]+corr_gripper_z 
            dy1 = dynamic_y_offset(y1,z1)
            dz1 =  dynamic_z_offset(y1,z1)

            # Apply correction offsets to the transform of the ArUco marker and set it as the new target pose
            # NOTE: Whhile callibrating, uncomment the second line 
            Zpose = [x1-dx1, y1-dy1, z1-dz1, Zpose[3], Zpose[4], Zpose[5], Zpose[6]]
            #Zpose = [transAruco[0]+d_cam_corr_x+corr_gripper_x, transAruco[1]+d_cam_corr_y+corr_gripper_y, transAruco[2]+corr_gripper_z, Zpose[3], Zpose[4], Zpose[5], Zpose[6]]
            
            
            print(Zpose)                                                        # Print pose just as a control measure 
            print("Va a moverse")
            move_joints2(2)                                                     # Move to focal point as a control measure 
            arm.set_target_pose(pose=Zpose, wait=True, t=1.0)                   # Then move to objective
            print("Terminó de moverse")

            return                                                              # Finish function

        # Handle exceptions related to transforms
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            #print("NO se va a mover. NO encontró tf")
            continue
        rate.sleep()

#===============================================================================#
# SECTION: Offset Functions                                                     #
# DESCRIPTION: These functions provide dynamic offsets based on given           #
#              coordinates, likely for compensating positional inaccuracies     #
#              in a robotic arm control system.                                 #
# INPUTS: x, y, z coordinates and optional parameters for offset calculations   #
# OUTPUTS: Adjusted coordinate values                                           #
#===============================================================================#
def dynamic_x_offset(x_coordinate, i1 = 0.009, i2 =-0.117, i3 =-9.85 ):
    # Function to dynamically offset the x-coordinate based on given parameters:
    # param x_coordinate: The original x-coordinate (in meters)
    # param i1: Coefficient for the quadratic term (default: 0.009)
    # param i2: Coefficient for the linear term (default: -0.117)
    # param i3: Constant term (default: -9.85)
    # return: Adjusted x-coordinate value (in meters)

    x_coordinate = x_coordinate*100
    return ((i1*x_coordinate*x_coordinate + i2*x_coordinate + i3)/100)

def dynamic_y_offset(y_coordinate, z_coordinate, j1 = 0.345, j2 = -0.186, j3 =4.054):
    # Function to dynamically offset the y-coordinate based on given parameters:
    # param y_coordinate: The original y-coordinate (in meters)
    # param z_coordinate: The original z-coordinate (in meters)
    # param j1: Coefficient for the y-coordinate term (default: 0.345)
    # param j2: Coefficient for the z-coordinate term (default: -0.186)
    # param j3: Constant term (default: 4.054)
    # return: Adjusted y-coordinate value (in meters)

    y_coordinate = y_coordinate*100
    z_coordinate = z_coordinate*100
    return ((j1*y_coordinate + j2* z_coordinate+ j3)/100)

def dynamic_z_offset(y_coordinate, z_coordinate, k1 = 0.216, k2 = 0.178, k3 =-6.764 ):
    # Function to dynamically offset the z-coordinate based on given parameters:
    # param y_coordinate: The original y-coordinate (in meters)
    # param z_coordinate: The original z-coordinate (in meters)
    # param k1: Coefficient for the y-coordinate term (default: 0.216)
    # param k2: Coefficient for the z-coordinate term (default: 0.178)
    # param k3: Constant term (default: -6.764)
    # return: Adjusted z-coordinate value (in meters)

    z_coordinate = z_coordinate*100
    y_coordinate = y_coordinate*100
    return ((k1*y_coordinate+k2*z_coordinate + k3)/100)

#===============================================================================#
# SECTION: Calibration                                                          #
# DESCRIPTION: This function performs a calibration process, moving the         #
#              robotic arm to specific positions and pausing between movements. #
# INPUTS: N/A                                                                   #
# OUTPUTS: N/A                                                                  #
#===============================================================================#
def callibration():
    for i in range(3):
        move_joints()
        print("Point %i:" % i)
        move_endeffector()
        rospy.sleep(30.0)
    move_joints()

#===============================================================================#
# SECTION: Routine                                                              #
# DESCRIPTION: This function performs a routine of movements and operations,    #
#              controlling both the arm joints and end-effector, and managing   #
#              the digital output state related to the robotic gripper.         #
# INPUTS: N/A                                                                   #
# OUTPUTS: N/A                                                                  #
#===============================================================================#
def routine():
    digital_output_manager = DigitalOutputManager([0.0, 0.0, 0.0])
    for i in range(4):
        move_joints()
        move_endeffector()
        print("CERRANDO gripper")
        rospy.sleep(1.0)
        digital_output_manager.restart_toggling()       
        print("TERMINÓ gripper")
        rospy.sleep(1.0)
        move_joints2(2)
        move_joints2(1)
        print("ABRIENDO gripper")
        rospy.sleep(1.0)
        digital_output_manager.restart_toggling()       
        print("TERMINÓ gripper")
    move_joints()

#===============================================================================#
# FUNCTION: main                                                                #
# DESCRIPTION: This function is the main entry point for this script. It takes  #
#              command line arguments for moving the robot arm and initializing #
#              the arm object. It measures execution time for both real and ROS #
#              system clocks.                                                   #
# INPUTS: Command-line arguments                                                #
#         -m/--move: If present, the robot will move to a joint configuration   #
#         -e/--move_ee: If present, the robot end effector will be moved        #
#         -c/--callibrate: If present, the robot will move to 3 points to allow #
#                         calibration.                                          #     
#         -r/--routine: If present, the robot will perform a full presentation  #
#                         routine.                                              #
# OUTPUTS: N/A                                                                  #
# SIDE EFFECTS: Moves the robotic arm, prints the execution time                #
#===============================================================================#
def main():
    """ Main function to be run. """
     # Instantiate an ArgumentParser object
    parser = argparse.ArgumentParser(description='Test force control')
    

    # Define command line arguments for the script
    parser.add_argument('-m', '--move', action='store_true',
                        help='move to joint configuration')

    parser.add_argument('-e', '--move_ee', action='store_true',
                        help='move to a desired end-effector position')
   
    parser.add_argument('-c', '--callibrate', action='store_true',
                        help='callibrate error in relation to a plane')
    
    parser.add_argument('-r', '--routine', action='store_true',
                        help='show final demonstration')
    # Parse the command line arguments
    args = parser.parse_args()

    # Initialize the ROS node with name 'ur3e_script_control'
    rospy.init_node('ur3e_script_control')
    
    # Initialize the global 'arm' variable with an instance of Arm class
    global arm
    arm = Arm(
        ft_sensor=True,                                                         # get Force/Torque data or not
        gripper=False,                                                          # Enable gripper
    )
    
    # Record the start time of execution in both real and ROS time
    real_start_time = timeit.default_timer()
    ros_start_time = rospy.get_time()

    # Change execution depending on the parameters
    if args.move:
        move_joints()

    if args.move_ee:
        move_endeffector()

    if args.callibrate:
        callibration()

    if args.routine:
        routine()

    # Print the elapsed real time and ROS time
    print("real time", round(timeit.default_timer() - real_start_time, 3))
    print("ros time", round(rospy.get_time() - ros_start_time, 3))


if __name__ == "__main__":
    main()