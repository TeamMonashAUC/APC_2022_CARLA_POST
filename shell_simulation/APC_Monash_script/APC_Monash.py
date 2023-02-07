#!/usr/bin/env python

'''
School: Monash University Malaysia

Written by:     Team Monash SEM Intelligent Department
   			 1) Andrew Joseph Ng Man Loong

Description: - Team Monash Shell Eco Marathon Autonomouse Programming Challenge Research code
   		 
'''

#################################################################################################################################################
# install instruction

# library name - "install command in linux terminal" (link to the library documentation)

'''
simple pid  -  "pip install simple-pid"  (https://pypi.org/project/simple-pid/)
'''


################################################################################################################################################# 
# import other prgramming files

import settings  # settings.py is used to store all global variables between files

import ROS_Communication     # does communications with rostopics & roscore (Level 1 code)
import Movement_Control   	 # utilise PID for throttle & linear steering using maximum turning angle by the car (Level 2 code)
import Coordinate_System   	 # move car to coordinate points on the map (Level 3 code)

#################################################################################################################################################
# import libraries
import rospy
import math
import numpy as np

#################################################################################################################################################

def main():
    # '''
    # "spawn_point": {"x": 113.6, "y": 132.2, "z": 8.2, "roll": 0.0, "pitch": 0.0, "yaw": 0.0},
    
    # Right turn test
    Coordinate_System.travel_to(15, [132.5, 132])
    Coordinate_System.corner(15,5,0,[150.2, 113.8],-90)

    Coordinate_System.travel_to(15, [149.9, 85.5])
    Coordinate_System.corner(15,5,-90,[137.9, 75.7],-180)

    Coordinate_System.travel_to(15, [96.1, 76.9])
    Coordinate_System.corner(15,5,-180,[83.4, 87.7],90)

    Coordinate_System.travel_to(15, [84.6, 113.5])
    Coordinate_System.corner(15,5,90,[97.0, 132.4],0)

    # Left turn test
    Coordinate_System.travel_to(15, [132.5, 132])
    Coordinate_System.corner(15,5,0,[154.5, 149.7],90)

    Coordinate_System.travel_to(15, [155, 179])
    Coordinate_System.corner(15,5,90,[135.8, 201.1],180)

    Coordinate_System.travel_to(15, [98.8, 202])
    Coordinate_System.corner(15,5,-180,[82.8,184],-90)


    Coordinate_System.travel_to(15, [82.5, 150.5])
    Coordinate_System.corner(15,5,-90,[99.6,132.8],0)
    # '''










    '''
	#   "spawn_point": {"x": -77.9, "y": -17.59, "z": 0.2, "roll": 0.0, "pitch": 0.0, "yaw": 90.0},
    Coordinate_System.travel_to(15, [-100, -0.4])
    corner(15,5,[-88.2, -32.9],-90)

    Coordinate_System.travel_to(45, [-84.8, -108.4])
    corner(15,5,[-59.7, -135.7],0)
    '''

    # Coordinate_System.travel_to(15, [-100, -0.4])
    # corner(15,5,[-74.3, 10.6],90)
    '''
    # first corner (working)
    Coordinate_System.travel_to(15, [-77.9, -15])
    corner(15,5,[-44.2, -1],0)
    '''
    
    '''
    # corner 2, (corner 1 working, corner 2 failed)
    Coordinate_System.travel_to(50, [-74.7, 84.1])
    Coordinate_System.travel_to(15, [-74.7, 104.2])
    corner(15,5,[-53.3,132.4],-90)
    
    Coordinate_System.travel_to(15, [-39.7, 131.7])
    corner(15,8,[-4.5,114.7],0)
    '''

    '''
    Coordinate_System.travel_to(15, [-77.9, -10])
    corner(15,5,[-108.7, 3.5],90)
    '''
    # corner(15,5,[10,10],-90)
    # corner(15,8,[-53.3,131.6],-90)
    # travel_to(15, [-44.2, -1])


    val=0
    while not rospy.is_shutdown():
        rospy.ROSInterruptException  # allow control+C to exit the program
        # rospy.loginfo(goal_position_from_car([-56.9,-0.4]))
        # corner(20,5,[10,10],60)


        # val = val +1
        # rospy.loginfo("total run = " + str(val))
        
        Movement_Control.carControl(targetSpeed = 0,steerAngle = 0)



    

    





#################################################################################################################################################
try:
    # single time setup

    # start rosnode
    rospy.init_node('APC_Monash')
    rate = rospy.Rate(50) # publish data at 50Hz
    rospy.loginfo("APC_Monash started")

    # start ros communications with rostopics
    ROS_Communication.ROS_Start()

    while not rospy.is_shutdown():
        # infinite loop
        main()
        rate.sleep()

except rospy.ROSInterruptException: # if we stop the script (using CTRL+C), it will run rospy.ROSInterruptException
    
    rospy.loginfo("Exit program successful") # Exit message
    pass
