#!/usr/bin/env python

'''
School: Monash University Malaysia

Written by:     Team Monash SEM Intelligent Department
   			 1) Andrew Joseph Ng Man Loong
             2) Nulan Dammage 
             3) Lucas Wee
             4) Derrick Teoh
             5) Lai Qi Shiun


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
# from shell_simulation.ROS_Communication import test
from shell_simulation.ROS_Communication import ROS_Start  # does communications with rostopics & roscore (Level 1 code)
import shell_simulation.Movement_Control as Movement_Control   	 # utilise PID for throttle & linear steering using maximum turning angle by the car (Level 2 code)
import shell_simulation.Coordinate_System as Coordinate_System  	 # move car to coordinate points on the map (Level 3 code)

#################################################################################################################################################
# import libraries
import rospy
import math
import numpy as np

#################################################################################################################################################

def R1():
    Coordinate_System.travel_to(15, [-206.4, 4.2]) # P1
    Coordinate_System.travel_to(15, [-245.5,0.8]) 
    Coordinate_System.travel_to(15, [-255.9,0.2]) # P2   #stop

    Coordinate_System.corner(15,8,180,[-272.5,-18.7],-90) #turn left
    Coordinate_System.travel_to(15, [-272.1,-43.9]) # P3
    Coordinate_System.travel_to(15, [-272.0,-68.2]) 

    # curve
    Coordinate_System.travel_to(15, [-271.3,-75.5]) 
    Coordinate_System.travel_to(15, [-268.2,-80.7]) 
    Coordinate_System.travel_to(15, [-264.6,-84.3]) 
    Coordinate_System.travel_to(15, [-253.8,-89.6]) 
    Coordinate_System.travel_to(15, [-241.8,-91.2]) 

    Coordinate_System.travel_to(15, [-230.0,-94.9]) # P4
    Coordinate_System.travel_to(15, [-205.5,-95.0]) # P4
    Coordinate_System.travel_to(15, [-205.3,-94.7]) # traffic light 

def R2():
    # curve
    Coordinate_System.travel_to(15, [-195.0,-118.3]) 
    Coordinate_System.travel_to(15, [-193.7,-127.2]) 
    Coordinate_System.travel_to(15, [-191.8,-132.4]) 
    Coordinate_System.travel_to(15, [-183.9,-144.2]) 
    Coordinate_System.travel_to(15, [-175.5,-150.1]) 
    # Coordinate_System.travel_to(15, [-165.0,-153.9]) 

    Coordinate_System.travel_to(15, [-158.0,-151.1]) 
    Coordinate_System.travel_to(15, [-151.0,-151.0]) #P6 
    Coordinate_System.travel_to(15, [-145.6,-151.0]) # traffic light 


    Coordinate_System.travel_to(15, [-121.0,-151.1]) 
    Coordinate_System.travel_to(15, [-101.4,-154.7]) # P7

    #curve
    Coordinate_System.travel_to(15, [-85.8,-154.9]) 
    Coordinate_System.travel_to(15, [-75.0,-152.7]) 
    Coordinate_System.travel_to(15, [-65.8,-148.7]) 
    Coordinate_System.travel_to(15, [-54.4,-136.6]) 

    Coordinate_System.travel_to(15, [-47.8,-117.2]) # P8 
    Coordinate_System.travel_to(15, [-47.4,-105.9]) 


def main():
    R1()
    Coordinate_System.corner(15,6,0,[-195.0,-112.5],-90) #turn right
    R2()




    while not rospy.is_shutdown():
        rospy.ROSInterruptException  # allow control+C to exit the program        
        Movement_Control.carControl(targetSpeed = 0,steerAngle = 0)



    

    





#################################################################################################################################################
if __name__ == '__main__':
    try:
        # single time setup
        # test()
        # start rosnode
        rospy.init_node('APC_Monash')
        rate = rospy.Rate(100) # publish data at 100Hz
        rospy.loginfo("APC_Monash started")

        # start ros communications with rostopics
        ROS_Start()

        while not rospy.is_shutdown():
            # infinite loop
            main()
            rate.sleep()

    except rospy.ROSInterruptException: # if we stop the script (using CTRL+C), it will run rospy.ROSInterruptException
        
        rospy.loginfo("Exit program successful") # Exit message
        pass
