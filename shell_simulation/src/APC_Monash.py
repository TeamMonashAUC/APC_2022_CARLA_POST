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
    Coordinate_System.travel_to(40, [-206.4, 4.2]) # P1
    Coordinate_System.travel_to(40, [-245.5,0.8]) 
    Coordinate_System.travel_to(20, [-255.9,0.2]) # P2   #stop

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

def R3():
    Coordinate_System.travel_to(15, [-83.20,-87.70]) #P32
    Coordinate_System.travel_to(15, [-105.9,-87.8]) 
    
    Coordinate_System.corner(15,5,180,[-124.1,-72.3],90) #turn right
    Coordinate_System.travel_to(15, [-124.30,-42.40]) # P33
    Coordinate_System.travel_to(15, [-124.50,-17.5]) 
    
def R4():
    pass

def R5():
    # "spawn_point": {"x": -124.9, "y": 55.5, "z": 0.2, "roll": 0.0, "pitch": 0.0, "yaw": 90.0},
    Coordinate_System.travel_to(15, [-124.50,73.0]) 
    Coordinate_System.corner(15,5,90,[-106.2,88.1],0) #turn right
    Coordinate_System.travel_to(15, [-65.8,87.9]) 
    Coordinate_System.travel_to(15, [-33.5,87.9]) 

    Coordinate_System.travel_to(15, [-20.70,87.90]) # P37
    Coordinate_System.travel_to(15, [16,87.8]) 

def R6():
    Coordinate_System.travel_to(15, [47.9,146.0]) 
    Coordinate_System.travel_to(15, [84.70,144.10]) #P26

    #curve
    Coordinate_System.travel_to(15, [94,142.7]) 
    Coordinate_System.travel_to(15, [103.8,141.1]) 
    Coordinate_System.travel_to(15, [114.3,138.8]) 
    Coordinate_System.travel_to(15, [124.4,134.7]) 
    Coordinate_System.travel_to(15, [132.8,129.8]) 
    Coordinate_System.travel_to(15, [138.6,125.0]) 
    Coordinate_System.travel_to(15, [143.1,119.7]) 
    Coordinate_System.travel_to(15, [146.1,115.1]) 
    Coordinate_System.travel_to(15, [148.10,112.20]) #P27 
    Coordinate_System.travel_to(15, [150.0,106.60]) 
    Coordinate_System.travel_to(15, [150.8,101.40]) 
    Coordinate_System.travel_to(15, [151.3,94.4]) 
    Coordinate_System.travel_to(15, [151.4,87.1]) 

    Coordinate_System.travel_to(15, [151.4,13.2]) 
    Coordinate_System.corner(15,5,-90,[141.4,2.2],-180) #turn right
    Coordinate_System.travel_to(15, [124.70,1.9]) #P29
    Coordinate_System.travel_to(15, [116.5,2.2]) 

def R7():
    Coordinate_System.travel_to(15, [96.2,-14.9])
    Coordinate_System.travel_to(15, [96.20,-28.60]) #P30

    #curve
    Coordinate_System.travel_to(15, [95.70,-43.60]) 
    Coordinate_System.travel_to(15, [95,-52]) 
    Coordinate_System.travel_to(15, [92.8,-59.7]) 
    Coordinate_System.travel_to(15, [88.8,-67.3]) 
    Coordinate_System.travel_to(15, [83.4,-73.5]) 
    Coordinate_System.travel_to(15, [76.3,-79]) 
    Coordinate_System.travel_to(15, [65.9,-83.4]) 
    Coordinate_System.travel_to(15, [55.2,-84.9]) 

    Coordinate_System.travel_to(15, [45.90,-84.90]) #P30
    Coordinate_System.travel_to(15, [45.30,-84.90]) 

def R8(): 
    Coordinate_System.travel_to(15, [-32.8,-88.0]) #P31

def R9():
    Coordinate_System.travel_to(15, [-43.7,-73.9]) 
    Coordinate_System.travel_to(15, [-43.80,-56.80]) #P9
    Coordinate_System.travel_to(15, [-43.5,-16.9]) #P10

    Coordinate_System.corner(15,5,90,[-33.9,-2.6],0) #turn right
    Coordinate_System.travel_to(15, [3.00,-2.70]) #P11
    Coordinate_System.travel_to(15, [14.40,-2.60]) 
    

def R10():
    Coordinate_System.travel_to(15, [24.8,-19.3]) 
    Coordinate_System.travel_to(15, [24.60,-30.70]) #P39
    Coordinate_System.travel_to(15, [24.4,-72.3]) 


def R11():
    Coordinate_System.travel_to(15, [44.1,-1.9]) 
    Coordinate_System.travel_to(15, [47.80,-1.80]) #P12
    Coordinate_System.travel_to(15, [66.4,-5.2]) 
    Coordinate_System.travel_to(15, [89.00,-5.50]) #P13

def R12():
    Coordinate_System.travel_to(15, [25.70,65.40]) #P38
    Coordinate_System.travel_to(15, [27.8,15.8]) 

def R13():

    Coordinate_System.travel_to(15, [31.6,15.4]) 
    Coordinate_System.travel_to(15, [31.30,19.30]) #P15

    Coordinate_System.travel_to(15, [35.4,46.2]) 
    Coordinate_System.travel_to(15, [36.30,67.2]) #P16
    Coordinate_System.travel_to(15, [36.5,74.5]) 

def R14():
    pass
def main():
    Coordinate_System.travel_to(-15, [-171.60,4.00]) 
    R1()
    Coordinate_System.corner(15,6,0,[-195.0,-112.5],-90) #turn right
    R2()
    Coordinate_System.corner(15,6,90,[-65.8,-87.9],0) #turn left
    R3()
    if (False):
        R4()
    else:
        R5()

    Coordinate_System.corner(15,5,0,[29.4,75.5],-90) #turn right
    R12()
    R10()
    Coordinate_System.corner(15,5,-90,[14.7,-88],-180) #turn right
    R8()
    Coordinate_System.corner(15,5,-180,[-43.7,-73.9],90) #turn right
    R9()
    R11()
    Coordinate_System.corner(15,5,0,[96.2,-14.9],-90) #turn right
    R7()
    Coordinate_System.corner(15,5,-180,[31.7,-68.7],90) #turn right
    Coordinate_System.travel_to(15, [31.7,-16.2]) 
    R13()

    Coordinate_System.travel_to(15, [37.2,105.6]) 
    Coordinate_System.travel_to(15, [38,132.2]) 
    Coordinate_System.corner(15,5,90,[47.9,146.0],0) #turn right

    R6()

    Coordinate_System.travel_to(15, [87.6,5.1]) 
    Coordinate_System.travel_to(15, [44.7,5.2]) 
    Coordinate_System.corner(15,5,180,[31.6,15.4],90) #turn right

    R13()
    Coordinate_System.travel_to(15, [37.2,105.6]) 
    Coordinate_System.travel_to(15, [38,132.2]) 
    Coordinate_System.travel_to(15, [39.1,172.8]) 






    while not rospy.is_shutdown():
        rospy.ROSInterruptException  # allow control+C to exit the program        
        Movement_Control.carControl(targetSpeed = 0,steerAngle = 0)
        rate.sleep()
        # rospy.spin()



    

    





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
