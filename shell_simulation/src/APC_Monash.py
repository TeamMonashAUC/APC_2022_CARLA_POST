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
    Coordinate_System.travel_to(30, [-206.4, 4.2]) # P1
    Coordinate_System.travel_to(30, [-245.5,0.8]) 
    Coordinate_System.travel_to(20, [-255.9,0.2]) # P2   #stop

    Coordinate_System.corner(20,7,180,[-272.5,-18.7],-90) #turn left
    Coordinate_System.travel_to(20, [-272.1,-43.9]) # P3
    Coordinate_System.travel_to(20, [-272.0,-68.2]) 

    # curve
    Coordinate_System.travel_to(20, [-271.3,-75.5]) 
    Coordinate_System.travel_to(20, [-268.2,-80.7]) 
    Coordinate_System.travel_to(20, [-264.6,-84.3]) 
    Coordinate_System.travel_to(20, [-253.8,-89.6]) 
    Coordinate_System.travel_to(25, [-241.8,-91.2]) 

    Coordinate_System.travel_to(25, [-219.0,-94.6]) # P4
    Coordinate_System.travel_to(25, [-205.5,-95.0]) # P4
    Coordinate_System.travel_to(25, [-205.3,-94.7]) # traffic light 

def R2():
    # curve
    Coordinate_System.travel_to(25, [-199.1,-95.0])
    Coordinate_System.travel_to(25, [-195.0,-104.6]) 
    Coordinate_System.travel_to(25, [-195.0,-118.3]) 
    Coordinate_System.travel_to(25, [-193.7,-127.2]) 
    Coordinate_System.travel_to(25, [-191.8,-132.4]) 
    Coordinate_System.travel_to(25, [-183.9,-144.2]) 
    Coordinate_System.travel_to(25, [-175.5,-150.1]) 
    # Coordinate_System.travel_to(15, [-165.0,-153.9]) 

    Coordinate_System.travel_to(25, [-158.0,-151.1]) 
    Coordinate_System.travel_to(25, [-151.0,-151.0]) #P6 
    Coordinate_System.travel_to(35, [-145.6,-151.0]) # traffic light 


    Coordinate_System.travel_to(35, [-121.0,-151.1]) 
    Coordinate_System.travel_to(25, [-101.4,-154.7]) # P7

    #curve
    Coordinate_System.travel_to(25, [-85.8,-154.9]) 
    Coordinate_System.travel_to(25, [-75.0,-152.7]) 
    Coordinate_System.travel_to(25, [-65.8,-148.7]) 
    Coordinate_System.travel_to(25, [-54.4,-136.6]) 

    Coordinate_System.travel_to(25, [-47.8,-117.2]) # P8 
    Coordinate_System.travel_to(25, [-47.4,-105.9]) 

def R3():
    Coordinate_System.travel_to(25, [-47,-49.4]) 
    Coordinate_System.travel_to(25, [-43.8,-38.4]) 
    Coordinate_System.travel_to(25, [-43.1,-19.4]) #P10
    
def R4():
    Coordinate_System.travel_to(25, [-42.9,-11]) 
    Coordinate_System.travel_to(25, [-46.6,0.2]) 
    Coordinate_System.travel_to(25, [-55.5,4.4])
    Coordinate_System.travel_to(25, [-61.6,4.5])
    Coordinate_System.travel_to(25, [-84.2,4.3])
    Coordinate_System.travel_to(25, [-109.2,1]) # turn right
    Coordinate_System.travel_to(25, [-113.1,0.9])
    Coordinate_System.travel_to(25, [-117.4,1.1])
    Coordinate_System.travel_to(25, [-119.8,2.2])
    Coordinate_System.travel_to(25, [-120.7,3.9]) #p10
    Coordinate_System.travel_to(25, [-121.5,7.9])
    Coordinate_System.travel_to(25, [-121.2,73.8])
    Coordinate_System.travel_to(25, [-121.7,85.2])
    Coordinate_System.travel_to(25, [-121.7,95.1]) 
    Coordinate_System.travel_to(25, [-123.6,104.2])
    Coordinate_System.travel_to(25, [-124.4,112.8]) 
    Coordinate_System.travel_to(25, [-120.7,125.4]) # turn right
    Coordinate_System.travel_to(25, [-118.9,130.3])
    Coordinate_System.travel_to(25, [-117.9,132.8])
    Coordinate_System.travel_to(25, [-113.6,135.4])
    Coordinate_System.travel_to(25, [-110.8,135.2])
    Coordinate_System.travel_to(25, [-101.9,135])
    Coordinate_System.travel_to(25, [-88.9,135])
    Coordinate_System.travel_to(25, [-83.3,133.9])
    Coordinate_System.travel_to(25, [-75.8,132.3])
    Coordinate_System.travel_to(25, [-67.7,126.6])
    Coordinate_System.travel_to(25, [-62.5,121.1]) 
    Coordinate_System.travel_to(25, [-57.5,112])
    Coordinate_System.travel_to(25, [-55.8,105.1])

def R5():
    speed = 20
    Coordinate_System.travel_to(speed, [-51.4,93])
    Coordinate_System.travel_to(speed, [-51.6,74.5]) 
    Coordinate_System.travel_to(speed, [-51.1,13.8]) # turn left
    Coordinate_System.travel_to(speed, [-51,2.2])
    Coordinate_System.travel_to(speed, [-48.5,-1.3])
    Coordinate_System.travel_to(speed, [-41,-2.8]) 
    Coordinate_System.travel_to(speed, [-19.6,-2.7]) 
    Coordinate_System.travel_to(speed, [15.4,-2.5]) #P11 
    Coordinate_System.travel_to(speed, [15.4,-2.5]) 

def R6():
    speed = 20
    Coordinate_System.travel_to(speed, [43.1,-2]) #P12
    Coordinate_System.travel_to(speed, [63.2,-1.9]) 
    Coordinate_System.travel_to(speed, [87.9,-1.9]) # turn right 
    Coordinate_System.travel_to(speed, [94.4,-1.9]) 
    Coordinate_System.travel_to(speed, [96.7,-4.6]) 
    Coordinate_System.travel_to(speed, [97,-9.2])
    Coordinate_System.travel_to(speed, [96,-43.5])
    Coordinate_System.travel_to(speed, [94.3,-55.1])   
    Coordinate_System.travel_to(speed, [91.3,-62.9])  
    Coordinate_System.travel_to(speed, [85.2,-71.8])
    Coordinate_System.travel_to(speed, [74.1,-80.4]) #P14 
    Coordinate_System.travel_to(speed, [60.9,-84.4])   
    Coordinate_System.travel_to(speed, [43.7,-85.2]) # end   

def R7():
    speed = 20
    Coordinate_System.travel_to(speed, [40.2,-84.9])
    Coordinate_System.travel_to(speed, [38,-83.2])
    Coordinate_System.travel_to(speed, [36.3,-79.6])
    Coordinate_System.travel_to(speed, [35.6,-76.3])
    Coordinate_System.travel_to(speed, [35.4,-70.8])
    Coordinate_System.travel_to(speed, [35,-42.7]) 
    Coordinate_System.travel_to(speed, [34.9,-16.1])
    Coordinate_System.travel_to(speed, [34.9, 14.1])
    Coordinate_System.travel_to(speed, [31.3, 21.1]) # P15
    Coordinate_System.travel_to(speed, [31.8, 35.1])
    Coordinate_System.travel_to(speed, [33.0, 73.1]) # end
     

def main():
    R1()
    R2()
    R3()
    R4()
    R5()
    R6()
    R7()



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
    rate = rospy.Rate(100) # publish data at 100Hz
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
