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
    Coordinate_System.travel_to(20, [-206.4, 4.2]) # P1
    Coordinate_System.travel_to(20, [-245.5,0.8]) 
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
    Coordinate_System.travel_to(25, [-83.20,-87.70]) #P32
    Coordinate_System.travel_to(20, [-105.9,-87.8]) 
    
    Coordinate_System.corner(15,5,180,[-124.1,-72.3],90) #turn right
    Coordinate_System.travel_to(20, [-124.30,-42.40]) # P33
    Coordinate_System.travel_to(30, [-124.50,-17.5])
    Coordinate_System.travel_to(40, [-124.50,73.0])  
    
def R4():
    Coordinate_System.travel_to(40, [-124.4,106.3])
    Coordinate_System.travel_to(30, [-124.4,123.3]) #P35
    Coordinate_System.travel_to(25, [-123.9,136.8])
    Coordinate_System.travel_to(25, [-107.6,138.7])
    Coordinate_System.travel_to(25, [-96.6,138.6])
    Coordinate_System.travel_to(25, [-82.6,134.5])
    Coordinate_System.travel_to(25, [-80.2,133.3]) #P36
    Coordinate_System.travel_to(25, [-75.7,131.8])
    Coordinate_System.travel_to(25, [-75.0,131.4])
    Coordinate_System.travel_to(25, [-71.7,129.6])
    Coordinate_System.travel_to(25, [-66.0,125.1])
    Coordinate_System.travel_to(25, [-60.5,118.5])
    Coordinate_System.travel_to(25, [-58.4,114.9])
    Coordinate_System.travel_to(25, [-55.7,106.1])
    Coordinate_System.travel_to(25, [-55.5,104.7])
    Coordinate_System.travel_to(25, [-54.8,97.8])
    Coordinate_System.travel_to(25, [-35.6,87.8])

    


def R5():
    Coordinate_System.travel_to(20, [-20.70,87.90]) # P37
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
    Coordinate_System.travel_to(30, [116.5,2.2]) 

def R7():
    Coordinate_System.travel_to(25, [96.2,-14.9])
    Coordinate_System.travel_to(25, [96.20,-28.60]) #P30

    #curve
    Coordinate_System.travel_to(25, [95.70,-43.60]) 
    Coordinate_System.travel_to(25, [95,-52]) 
    Coordinate_System.travel_to(25, [92.8,-59.7]) 
    Coordinate_System.travel_to(25, [88.8,-67.3]) 
    Coordinate_System.travel_to(25, [83.4,-73.5]) 
    Coordinate_System.travel_to(25, [76.3,-79]) 
    Coordinate_System.travel_to(25, [65.9,-83.4]) 
    Coordinate_System.travel_to(25, [55.2,-84.9]) 

    Coordinate_System.travel_to(15, [45.90,-84.90]) #P30
    Coordinate_System.travel_to(15, [45.30,-84.90]) 

def R8(): 
    Coordinate_System.travel_to(30, [-32.8,-88.0]) #P31

def R9():
    Coordinate_System.travel_to(30, [-43.7,-73.9]) 
    Coordinate_System.travel_to(25, [-43.80,-56.80]) #P9
    Coordinate_System.travel_to(20, [-43.5,-16.9]) #P10

    Coordinate_System.corner(20,5,90,[-33.9,-2.6],0) #turn right
    Coordinate_System.travel_to(20, [3.00,-2.70]) #P11
    Coordinate_System.travel_to(20, [14.40,-2.60]) 
    

def R10():
    Coordinate_System.travel_to(25, [24.8,-19.3]) 
    Coordinate_System.travel_to(25, [24.60,-30.70]) #P39
    Coordinate_System.travel_to(25, [24.4,-72.3]) 


def R11():
    Coordinate_System.travel_to(15, [44.1,-1.9]) 
    Coordinate_System.travel_to(15, [47.80,-1.80]) #P12
    Coordinate_System.travel_to(15, [66.4,-5.2]) 
    Coordinate_System.travel_to(15, [89.00,-5.50]) #P13

def R12():
    Coordinate_System.travel_to(25, [25.70,65.40]) #P38
    Coordinate_System.travel_to(25, [27.8,15.8]) 

def R13():

    Coordinate_System.travel_to(30, [31.6,15.4]) 
    Coordinate_System.travel_to(20, [31.30,19.30]) #P15

    Coordinate_System.travel_to(20, [35.4,46.2]) 
    Coordinate_System.travel_to(25, [36.30,67.2]) #P16
    Coordinate_System.travel_to(25, [36.5,74.5]) 

def R14():#Final ring

    Coordinate_System.travel_to(25, [50.0,190.2])
    Coordinate_System.travel_to(20, [52.0,190.5])
    Coordinate_System.travel_to(20, [54.0,190.5]) 
    Coordinate_System.travel_to(20, [74.0,190.5])   #P18 (74.0, 190.0)

    # Cutting to the side lane 
    Coordinate_System.travel_to(20, [95.1,193.7])
    Coordinate_System.travel_to(20, [95.5,193.7])
    Coordinate_System.travel_to(20, [108.0,193.7])
    Coordinate_System.travel_to(20, [115.8,192.5])
    Coordinate_System.travel_to(20, [122.0,191.5])
    Coordinate_System.travel_to(20, [126.0,190.5])
    Coordinate_System.travel_to(20, [131.5,188.8])
    Coordinate_System.travel_to(20, [135.0,187.5])
    Coordinate_System.travel_to(20, [140.0,185.5])
    Coordinate_System.travel_to(20, [145.7,182.2])
    Coordinate_System.travel_to(20, [148.1,181.0])  
    Coordinate_System.travel_to(20, [154.1,177.3])  #P19
    Coordinate_System.travel_to(20, [165.0,168.0])
    Coordinate_System.travel_to(20, [174.5,158.0])
    Coordinate_System.travel_to(20, [180.2,150.6])
    Coordinate_System.travel_to(20, [184.1,143.5])
    Coordinate_System.travel_to(20, [187.5,137.1])
    Coordinate_System.travel_to(20, [190.0,130.7])
    Coordinate_System.travel_to(20, [192.5,121.4])
    Coordinate_System.travel_to(20, [193.7,117.3])
    Coordinate_System.travel_to(20, [195.6,101.3])
    Coordinate_System.travel_to(20, [189.1,72.0])   #189.3?
    Coordinate_System.travel_to(20, [189.2,52.8])   #P20
    Coordinate_System.travel_to(30, [189.2,50.0])
    Coordinate_System.travel_to(30, [190.0,-4.0])
    Coordinate_System.travel_to(30, [189.5,-80.0])
    Coordinate_System.travel_to(30, [188.9,-104.7])
    Coordinate_System.travel_to(30, [187.4,-116.4])
    Coordinate_System.travel_to(30, [183.9,-129.5])
    Coordinate_System.travel_to(30, [181.5,-135.7])
    Coordinate_System.travel_to(30, [179.0,-140.0])
    Coordinate_System.travel_to(30, [175.5,-146.0])
    Coordinate_System.travel_to(30, [174.4,-148.0]) #P21
    Coordinate_System.travel_to(30, [168.5,-156.0])
    Coordinate_System.travel_to(30, [156.6,-168.0])
    Coordinate_System.travel_to(30, [143.5,-176.9])
    Coordinate_System.travel_to(30, [134.7,-180.9])
    Coordinate_System.travel_to(30, [123.1,-185.0])
    Coordinate_System.travel_to(30, [111.1,-187.4])
    Coordinate_System.travel_to(30, [104.2,-187.8])
    Coordinate_System.travel_to(40, [100.0,-187.9])
    Coordinate_System.decel(40, [95.6,-187.8])
    Coordinate_System.decel(40, [60,-187.8])
    Coordinate_System.decel(40, [10.2,-187.9]) #P22
    Coordinate_System.travel_to(40, [-60,-188.9]) 
    Coordinate_System.travel_to(40, [-145.8,-190.9]) #P23 
    Coordinate_System.travel_to(40, [-152.0,-190.7])
    Coordinate_System.decel(40, [-158.5,-189.7])
    Coordinate_System.decel(40, [-166.2,-187.9])
    Coordinate_System.travel_to(30, [-193.3,-177.6])
    Coordinate_System.travel_to(30, [-196.6,-175.9])
    Coordinate_System.travel_to(30, [-201.3,-172.6])
    Coordinate_System.travel_to(30, [-205.4,-169.2])
    Coordinate_System.travel_to(30, [-212.5,-161.2])
    Coordinate_System.travel_to(30, [-219.7,-150.9])
    Coordinate_System.travel_to(30, [-225.2,-141.4])
    Coordinate_System.travel_to(30, [-229.0,-132.3])
    Coordinate_System.travel_to(30, [-230.7,-125.9])
    Coordinate_System.decel(40, [-232.4,-118.1])
    Coordinate_System.decel(40, [-233.5,-111.5])
    Coordinate_System.decel(40, [-233.8,-103.8])
    Coordinate_System.decel(40, [-233.6,-84.3])
    Coordinate_System.decel(40, [-232.6,28.1]) #P24 
    Coordinate_System.travel_to(35, [-233.9,98.7])
    Coordinate_System.travel_to(30, [-224.3,132.8])
    Coordinate_System.travel_to(30, [-210.5,151.2])
    Coordinate_System.travel_to(30, [-187.9,176.2])
    Coordinate_System.travel_to(30, [-176.2,182.5])
    Coordinate_System.decel(40, [-163.7,186.8])
    Coordinate_System.decel(40, [-125.8,186.5])
    Coordinate_System.decel(40, [-119.4,186.6]) #P25 



def main():
    #R1()
    #R2()

    #Coordinate_System.corner(20,6,90,[-65.8,-87.9],0) #turn left
    #R3()
    #R4()
    #R5()
    #Coordinate_System.corner(20,5,0,[29.4,75.5],-90) #turn right
    #R12()

    #R10()
    #Coordinate_System.corner(15,5,-90,[14.7,-88],-180) #turn right
    #R8()
    #Coordinate_System.corner(15,5,-180,[-43.7,-73.9],90) #turn right
    #R9()
    #R11()
    #Coordinate_System.corner(15,5,0,[96.2,-14.9],-90) #turn right
    #R7()
    #Coordinate_System.corner(15,5,-180,[31.7,-68.7],90) #turn right
    #Coordinate_System.travel_to(15, [31.7,-16.2]) 
    #R13()

    #Coordinate_System.travel_to(25, [37.2,105.6]) 
    #Coordinate_System.travel_to(20, [38,132.2]) 
    #Coordinate_System.corner(15,5,90,[47.9,146.0],0) #turn right
                                                                                                                                                                                                                                                                      
    #R6()

    #Coordinate_System.travel_to(25, [87.6,5.1]) 
    #Coordinate_System.travel_to(20, [44.7,5.2]) 
    #Coordinate_System.corner(15,5,180,[31.6,15.4],90) #turn right

    #R13()
    Coordinate_System.travel_to(25, [37.2,105.6]) 
    Coordinate_System.travel_to(25, [38,132.2]) 
    Coordinate_System.travel_to(25, [39.1,172.8]) 

    Coordinate_System.travel_to(15, [39.1,180.2])
    
    Coordinate_System.corner(15,6,90,[48.3,190.2],0)
    R14()
    

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
