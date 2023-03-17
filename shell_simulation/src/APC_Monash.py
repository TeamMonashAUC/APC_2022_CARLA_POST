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

import shell_simulation_2.settings as settings # settings.py is used to store all global variables between files
# from shell_simulation.ROS_Communication import test
import shell_simulation_2.ROS_Communication as ROS_Communication  # does communications with rostopics & roscore (Level 1 code)
import shell_simulation_2.Movement_Control as Movement_Control   	 # utilise PID for throttle & linear steering using maximum turning angle by the car (Level 2 code)
import shell_simulation_2.Coordinate_System as Coordinate_System  	 # move car to coordinate points on the map (Level 3 code)

from shell_simulation.msg import ActualCoord

#################################################################################################################################################
# import libraries
import rospy
import math
import numpy as np

#################################################################################################################################################


def R1():
    # "spawn_point": {"x": -171.60, "y": 4.0, "z": 0.2, "roll": 0.0, "pitch": 0.0, "yaw": 180.0},
    Coordinate_System.travel_to(16, [-206.4, 4.2]) # P1
    Coordinate_System.travel_to(16, [-245.5,0.8]) 
    Coordinate_System.travel_to(16, [-255.9,0.2]) # P2   #stop

    Coordinate_System.corner(16,7,180,[-272.5,-18.7],-90) #turn left
    Coordinate_System.travel_to(16, [-272.1,-43.9]) # P3
    Coordinate_System.travel_to(16, [-272.0,-68.2]) 

    # curve
    Coordinate_System.travel_to(16, [-271.9,-68.7])
    Coordinate_System.travel_to(16, [-271.7,-71.2])
    Coordinate_System.travel_to(16, [-270.6,-75.5])
    Coordinate_System.travel_to(16, [-269.6,-78.1])
    Coordinate_System.travel_to(16, [-268.6,-79.7])  
    Coordinate_System.travel_to(16, [-268.2,-80.7]) 
    Coordinate_System.travel_to(16, [-264.6,-84.3]) 
    Coordinate_System.travel_to(16, [-253.8,-89.6])  ##
    Coordinate_System.travel_to(16, [-241.8,-91.2]) 

    Coordinate_System.travel_to(16, [-219.0,-94.6]) # P4
    Coordinate_System.travel_to(16, [-205.5,-95.0]) # P4
    Coordinate_System.travel_to(16, [-205.3,-94.7]) # traffic light 

def R2():
    # curve
    Coordinate_System.travel_to(16, [-199.1,-95.0])
    Coordinate_System.travel_to(16, [-195.0,-104.6]) 
    Coordinate_System.travel_to(16, [-195.0,-118.3]) 
    Coordinate_System.travel_to(16, [-193.7,-126.2]) 
    Coordinate_System.travel_to(16, [-191.8,-132.4])
    Coordinate_System.travel_to(16, [-189.4,-137.0])
    Coordinate_System.travel_to(16, [-185.5,-142.4]), #P5 #changed  x  
    Coordinate_System.travel_to(16, [-183.9,-144.2])
    Coordinate_System.travel_to(16, [-175.5,-150.1]) 
    # Coordinate_System.travel_to(15, [-165.0,-153.9]) 

    Coordinate_System.travel_to(16, [-158.0,-151.1]) 
    Coordinate_System.travel_to(16, [-151.0,-151.0]) #P6 
    Coordinate_System.travel_to(16, [-145.6,-151.0]) # traffic light 

def R3():
    Coordinate_System.travel_to(16, [-130.6,-151.0])
    Coordinate_System.travel_to(16, [-123.9,-139.1])
    Coordinate_System.travel_to(16, [-123.5,-135.5])
    Coordinate_System.travel_to(16, [-123.5,-132.9])
    Coordinate_System.travel_to(16, [-123.7,-106.4])
    Coordinate_System.travel_to(16, [-123.9,-74.9])
    Coordinate_System.travel_to(16, [-124.6,-16.7])
    Coordinate_System.travel_to(16, [-125.0,13.6])
    Coordinate_System.travel_to(16, [-124.6,72.7])
    Coordinate_System.travel_to(16, [-124.4,123.3]) #P35
    Coordinate_System.travel_to(16, [-123.9,129.0])
    Coordinate_System.travel_to(16, [-117.8,134.4])
    Coordinate_System.travel_to(16, [-107.6,135.1])
    Coordinate_System.travel_to(16, [-105.0,135.1])
    Coordinate_System.travel_to(16, [-96.6,135.0])
    Coordinate_System.travel_to(16, [-87.0,134.8])
    Coordinate_System.travel_to(16, [-84.0,134.5])
    Coordinate_System.travel_to(16, [-80.2,133.6]) #P36
    Coordinate_System.travel_to(16, [-75.7,131.8])
    Coordinate_System.travel_to(16, [-75.0,131.4])
    Coordinate_System.travel_to(16, [-71.7,129.6])
    Coordinate_System.travel_to(16, [-66.0,125.1])
    Coordinate_System.travel_to(16, [-60.5,118.5])
    Coordinate_System.travel_to(16, [-58.4,114.9])
    Coordinate_System.travel_to(16, [-55.7,106.1])
    Coordinate_System.travel_to(16, [-55.5,104.7])

def R4():
    Coordinate_System.travel_to(25, [-55.0,76.0])
    Coordinate_System.travel_to(25, [-54.8,15.0])
    Coordinate_System.travel_to(25, [-54.2,-14.9])

    Coordinate_System.travel_to(15, [-54.1,-18.1])
    Coordinate_System.travel_to(15, [-51.5,-22.3])
    Coordinate_System.travel_to(15, [-46.7,-23.3])
    Coordinate_System.travel_to(15, [-45.5,-22.7])
    Coordinate_System.travel_to(15, [-45.0,-21.3])
    Coordinate_System.travel_to(15, [-44.7,-20.9])
    Coordinate_System.travel_to(15, [-43.9,-17.5])
    Coordinate_System.travel_to(15, [-43.9,-17.1])
    Coordinate_System.travel_to(15, [-43.5,-10.5])
    

def R5(): # All 25 
    Coordinate_System.travel_to(20, [-35.8, -6.3])
    Coordinate_System.travel_to(25, [-33.1, -6.3])
    Coordinate_System.travel_to(25, [3.0, -2.7])
    Coordinate_System.travel_to(25, [15.9, -2.5])
    Coordinate_System.travel_to(25, [44.1, -1.9]) 
    Coordinate_System.travel_to(25, [47.80, -1.80]) #P12
    Coordinate_System.travel_to(30, [66.4, -5.2])
    Coordinate_System.travel_to(30, [88.0, -5.5])
    Coordinate_System.corner(15,5,0,[96.2,-14.9],-90) #turn right
    Coordinate_System.travel_to(25, [96.2, -28.6])
    Coordinate_System.travel_to(25, [95.7, -43.6]) 
    Coordinate_System.travel_to(25, [95.0, -52.0]) 
    Coordinate_System.travel_to(25, [92.8, -59.7]) 
    Coordinate_System.travel_to(25, [88.8, -67.3]) 
    Coordinate_System.travel_to(20, [83.4, -73.5]) 
    Coordinate_System.travel_to(25, [76.3, -79.0]) 
    Coordinate_System.travel_to(25, [65.9, -83.4]) 
    Coordinate_System.travel_to(25, [55.2, -84.9])
    Coordinate_System.travel_to(20, [45.9, -84.9]) # P14

def R6():
    Coordinate_System.travel_to(25, [39.9, -84.9])
    Coordinate_System.travel_to(25, [35.1, -76.2])
    Coordinate_System.travel_to(25, [35.1, -73.7])
    Coordinate_System.travel_to(25, [35.1, -16.1])
    Coordinate_System.travel_to(25, [35.0, 13.3])
    Coordinate_System.travel_to(25, [36.4, 72.7])
    Coordinate_System.travel_to(25, [37.2, 105.6]) 
    Coordinate_System.travel_to(20, [38.0, 132.2])

def R7():
    Coordinate_System.corner(20,5,90,[47.9,146.0],0) #turn right
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
    Coordinate_System.travel_to(20, [116.5,2.2]) 

def R8(): 
    Coordinate_System.travel_to(20, [87.9, 1.6])
    Coordinate_System.travel_to(20, [43.7, 1.8])
    Coordinate_System.travel_to(20, [39.4, 1.9])
    Coordinate_System.travel_to(20, [33.6, 5.9])
    Coordinate_System.travel_to(20, [31.5, 9.3])
    Coordinate_System.travel_to(20, [31.5, 13.8])
    Coordinate_System.travel_to(20, [31.6, 15.9])
    Coordinate_System.travel_to(20, [31.3, 19.3]) #P15
    Coordinate_System.travel_to(20, [33.0, 73.2])
    Coordinate_System.travel_to(20, [33.8, 103.6])
    Coordinate_System.travel_to(20, [38.6, 155.1]) # P17
    Coordinate_System.travel_to(20, [39.1, 175.8])    
    
def R9():
    Coordinate_System.travel_to(20, [39.3, 181.7])    

    # curve
    Coordinate_System.travel_to(20, [47.7, 190.1])    
    Coordinate_System.travel_to(20, [57.6,190.3])
    Coordinate_System.travel_to(20, [74.0,190.5])   #P18 (74.0, 190.0)

    Coordinate_System.travel_to(20, [100.9,190.4])
    Coordinate_System.travel_to(20, [112.6,189.6])
    Coordinate_System.travel_to(20, [124.9,187.0])
    Coordinate_System.travel_to(20, [137.6,182.7])
    Coordinate_System.travel_to(20, [146.1,180.2])
    Coordinate_System.travel_to(20, [151.8,178.3])
    # Coordinate_System.travel_to(0, [153.0,178.4])
    # Coordinate_System.travel_to(20, [154.10,177.30])  #P19

    Coordinate_System.travel_to(20, [154.10,176.30])  #P19

    # Coordinate_System.travel_to(20, [161.3,171.6]) 
    Coordinate_System.travel_to(20, [171.8,157.2])  
    Coordinate_System.travel_to(20, [182.8,132.3])  
    Coordinate_System.travel_to(20, [187.3,113.9])   

    # straight
    Coordinate_System.travel_to(20, [188.6,97.4])    
    Coordinate_System.travel_to(20, [189.2,51.9])     
    Coordinate_System.travel_to(20, [189.8,-31.6])   

    # curve
    Coordinate_System.travel_to(20, [188.9,-98.8]) 
    Coordinate_System.travel_to(20, [186.6,-119.9]) 
    Coordinate_System.travel_to(20, [180.6,-137.3]) 
    Coordinate_System.travel_to(20, [168.7,-155.7]) 
    Coordinate_System.travel_to(20, [153.8,-170.1]) 
    Coordinate_System.travel_to(20, [137.1,-180.0]) 
    Coordinate_System.travel_to(20, [118.5,-186.1]) 
    Coordinate_System.travel_to(20, [118.5,-186.1])     

    # straight
    Coordinate_System.travel_to(20, [98.8,-188.0])    
    Coordinate_System.travel_to(20, [46.6,-187.8])     
    Coordinate_System.travel_to(20, [10.2,-187.9]) #P22

    Coordinate_System.travel_to(20, [-76.7,-187.5])    
    Coordinate_System.travel_to(20, [-144.0,-187.5])      

    # curve
    Coordinate_System.travel_to(20, [-155.2,-186.4]) 
    Coordinate_System.travel_to(20, [-164.2,-184.6]) 
    Coordinate_System.travel_to(20, [-169.0,-183.0]) 
    Coordinate_System.travel_to(20, [-171.4,-182.1]) 
    Coordinate_System.travel_to(20, [-176.2,-180.2]) 
    Coordinate_System.travel_to(20, [-180.9,-178.0]) 
    Coordinate_System.travel_to(20, [-189.9,-172.4]) 
    Coordinate_System.travel_to(20, [-198.4,-165.7]) 
    Coordinate_System.travel_to(20, [-205.7,-158.5]) 
    Coordinate_System.travel_to(20, [-216.1,-144.1]) 
    Coordinate_System.travel_to(20, [-222.3,-129.7]) 
    Coordinate_System.travel_to(20, [-226.1,-113.3]) 

    # straight
    Coordinate_System.travel_to(20, [-232.5,-75.6]) 
    Coordinate_System.travel_to(20, [-232.6,-29.3]) 

    Coordinate_System.travel_to(20, [-232.6,28.1]) #P24 


def main():
    
    R1()
    R2()
    R3()
    R4()
    R5()
    R6()
    R7()
    R8()
    R9()

    while not rospy.is_shutdown():
        rospy.ROSInterruptException  # allow control+C to exit the program        
        Movement_Control.carControl(targetSpeed = 0,steerAngle = 0)
        rate.sleep()


    

    





#################################################################################################################################################
if __name__ == '__main__':
    # try:


    # single time setup
    # test()
    # start rosnode
    rospy.init_node('APC_Monash')
    rate = rospy.Rate(2) # publish data at 100Hz
    rospy.loginfo("APC_Monash started")

    # start ros communications with rostopics
    ROS_Communication.ROS_Start()


    # global pub_coord_2D
    # global pub_coord_3D 
    # pub_coord_3D = rospy.Publisher("/actual_coord_3D", ActualCoord, queue_size = 10)

    while not rospy.is_shutdown():
    #try: 
        ## Start procedure
        rospy.loginfo(settings.car_coordinate_from_world)
        while settings.car_coordinate_from_world[0] ==0:      
            Movement_Control.carControl(targetSpeed = 0,steerAngle = 0)
            # rospy.loginfo(settings.car_coordinate_from_world)
            pass
        rospy.loginfo("start coord:")
        rospy.loginfo(settings.car_coordinate_from_world)

        # infinite loop
        main()
        rate.sleep()
 
    # except ValueError:
    #     print("Oops!  That was no valid number.  Try again...")

    # except rospy.ROSInterruptException: # if we stop the script (using CTRL+C), it will run rospy.ROSInterruptException
        
    #     rospy.loginfo("Exit program successful") # Exit message
    #     pass
