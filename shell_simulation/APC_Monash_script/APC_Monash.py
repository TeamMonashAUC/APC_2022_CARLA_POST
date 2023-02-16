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
    """
    # 135 degree corner turn
    Coordinate_System.travel_to(15, [-77.9, -10])
    #Coordinate_System.corner(15,5,90,[-104.5,-20],-135)
    Coordinate_System.pointToPointCorner(15,4,90,[-104.5,-20],-135, 10)

    
<<<<<<< HEAD
    '''
    # test straight line function 1
    travel_to(80,[-71.6,150])
    travel_to(20,[-71.6,194])
=======
    Coordinate_System.travel_to(15, [-125.8, -42.1])
    Coordinate_System.pointToPointCorner(15,4,-135,[-142.2,-26.7],90, 10)
    #Coordinate_System.corner(15,5,-135,[-142.2,-26.7],90)

    """    
    # test with APC 2022 layout
    Coordinate_System.travel_to(15, [-77.9, -17.6])
    Coordinate_System.pointToPointCorner(15,10,90,[-50.9,-0.8],0)
    #Coordinate_System.corner(15,10,90,[-50.9,-0.8],0)
>>>>>>> 8594cd1ca82e45ea295be26f76d47090cddea269
    rospy.loginfo ("1")

    Coordinate_System.travel_to(20,[-41.4,-0.7])
    Coordinate_System.travel_to(20,[-29.2,-2.5])
    Coordinate_System.travel_to(20, [-23.3, -7.6])
    Coordinate_System.travel_to(20,[-21.7,-11.5])
    Coordinate_System.travel_to(20, [-16.4, -17.6])
    Coordinate_System.travel_to(20,[-12.3,-21])
    Coordinate_System.travel_to(20,[-8.3,-23.1])
    Coordinate_System.travel_to(20,[-4.1, -23.7])
    Coordinate_System.travel_to(20,[-1.77,-23.78])
    rospy.loginfo ("2")


    Coordinate_System.travel_to(20,[5.5, -23.3])
    Coordinate_System.travel_to(20,[8.7, -22.4])
    Coordinate_System.travel_to(20,[10.8,-20.30])
    Coordinate_System.travel_to(20,[12.5,-20])
    Coordinate_System.travel_to(20,[15.7,- 17.1])
    Coordinate_System.travel_to(20,[18.2,-14.40])
    Coordinate_System.travel_to(20,[20.5,-11])
    Coordinate_System.travel_to(20,[22.2, -9.1])
    Coordinate_System.travel_to(20,[24.1,-8.7])
    Coordinate_System.travel_to(20,[26.9,-7.79])

    Coordinate_System.travel_to(50,[30.6,-7.2])
    Coordinate_System.travel_to(50,[36.9,-7.3])
    Coordinate_System.travel_to(50, [45.2,-7.4])
    Coordinate_System.travel_to(50,[79.56,-7.79])
    rospy.loginfo ("3")
    

    Coordinate_System.travel_to(50,[192.6,-9.5])
    Coordinate_System.travel_to(20,[217.1,-9.9])
    Coordinate_System.pointToPointCorner(10,8,0,[231,-25.4],-90)
    #Coordinate_System.corner(10,8,0,[231,-25.4],-90)
    rospy.loginfo ("4")
<<<<<<< HEAD
    '''

    #  '''
    # test straight line function 3
    travel_to(80,[-78,120.4])
    travel_to(20,[-98,140.8])
    rospy.loginfo ("1")

    travel_to(80,[-122.6,135.7])
    travel_to(50,[-143.2,115.4])
    rospy.loginfo ("2")

    travel_to(80,[-149.4,12.5])
    travel_to(50,[-134.9,0.2])
    rospy.loginfo ("3")

    travel_to(80,[-96.8,0])
    travel_to(50,[-85.6,-22.2])
    rospy.loginfo ("4")

    travel_to(80,[-84,-120.6])
    travel_to(20,[-60,-136.1])
    travel_to(80,[129.9,-133.1])
    travel_to(20,[142.3,-130.8])
    travel_to(80,[157.9,-122.1])
    travel_to(80,[167.1,-108.1])
    travel_to(80,[170.9,-80.1])
    rospy.loginfo ("5")

    # Small U turn
    travel_to(80,[146.3,-58.70])
    travel_to(50,[105,-59])
    travel_to(80,[95,-56])
    travel_to(50,[80.4,-47.7])
    travel_to(80,[70,-49])
    travel_to(50,[64.4,-56.7])
    travel_to(80,[64.6,-64.5])
    travel_to(50,[74.4,-73.7])
    travel_to(80,[85.6,-71.5])
    travel_to(80,[108,-62.7])
    travel_to(80,[219,-62.7])
    rospy.loginfo ("6")

    # Tunnel
    travel_to(80,[241.8,-41.7])
    travel_to(80,[245.9,156])
    travel_to(80,[234.8,184.7])
    travel_to(80,[224.9,198])
    travel_to(80,[209.9,206])
    travel_to(80,[98.9,205.4])
    rospy.loginfo ("7")

    # slope & Roundabout
    travel_to(80,[79.9,183.6])
    travel_to(80,[74,20.4])
    travel_to(50,[63,8])
    travel_to(50,[25.5,8.5])
    travel_to(50,[1.5,21.5])
    travel_to(50,[-15.5,10.8])
    travel_to(50,[-20.5,-6.5])
    travel_to(80,[-11.5,-28.5])
    travel_to(80,[-9.3,-158.5])
    rospy.loginfo ("8")

    # Gas station
    travel_to(80,[-21.3,-170.5])
    travel_to(80,[-47.6,-187.5])
    travel_to(80,[-68.6,-184.5])
    travel_to(80,[-77.6,-146.5])
    travel_to(80,[-77.6,-12.5])
    rospy.loginfo ("9")
    # '''


    while not rospy.is_shutdown():
        Movement_Control.carControl(targetSpeed = 0,steerAngle = 0)
        rospy.ROSInterruptException  # allow control+C to exit the program

def corner(speed,endPos,turnRadius,target_angle):
    #1 get startPos,endPos and radius 
    startPos = [settings.car_coordinate_from_world[0],settings.car_coordinate_from_world[1]]
    # startAngle = settings.car_direction_from_world[2]
    
    # rospy.loginfo(startPos)
    # 2 calculate intersection point
    
    intersectPos = [startPos[0],endPos[1]]
    # rospy.loginfo(intersectPos)
    
    #3 find start curning coordinate
=======
>>>>>>> 8594cd1ca82e45ea295be26f76d47090cddea269

    Coordinate_System.travel_to(20,[231,-40.5])
    #Coordinate_System.corner(15,8,-90,[209.3,-58.6],180)
    Coordinate_System.pointToPointCorner(15,8,-90,[209.3,-58.6],180)
    rospy.loginfo ("5")

    Coordinate_System.travel_to(20,[188,-58.6])
    #Coordinate_System.corner(15,8,180,[167.4,-80.5],-90)
    Coordinate_System.pointToPointCorner(15,8,180,[167.4,-80.5],-90)
    rospy.loginfo ("6")
				

    #Coordinate_System.corner(15,32,-90,[116.3,-129.2],-180)
    Coordinate_System.pointToPointCorner(15,32,-90,[116.3,-129.2],-180)
    Coordinate_System.travel_to(50,[34.7,-130.1])
    Coordinate_System.travel_to(20,[18.6,-130.4])
    rospy.loginfo ("7")
    #'''

    #Coordinate_System.corner(15,8,180,[-6,-146.6],-90)
    Coordinate_System.pointToPointCorner(15,8,180,[-6,-146.6],-90)
    Coordinate_System.travel_to(20,[-5.6,-187.6])
    rospy.loginfo ("8")


    #Coordinate_System.corner(15,8,-90,[-25,-197.1],-180)
    #Coordinate_System.corner(15,32,-180,[-78.1,-149.1],90)
    Coordinate_System.pointToPointCorner(15,8,-90,[-25,-197.1],-180)
    Coordinate_System.pointToPointCorner(15,30,-180,[-78.1,-149.1],90, 100)
    rospy.loginfo ("9")


    #Coordinate_System.corner(15,8,90,[-98.7,-132.6],-180)
    #Coordinate_System.corner(15,28,-180,[-145.5,-80.8],90)		
    Coordinate_System.pointToPointCorner(15,8,90,[-98.7,-132.6],-180)
    Coordinate_System.pointToPointCorner(15,28,-180,[-145.5,-80.8],90)
    rospy.loginfo ("10")


    Coordinate_System.travel_to(20,[-145.75,-9.2])
    rospy.loginfo ("11")


    #Coordinate_System.corner(15,5,90,[-127.7,-0.3],0)	
    Coordinate_System.pointToPointCorner(15,5,90,[-127.7,-0.3],0)
    Coordinate_System.travel_to(20,[-107.6,-0.4])
    #Coordinate_System.corner(15,8,0,[-77.9,21.4],90)
    Coordinate_System.pointToPointCorner(15,8,0,[-77.9,21.4],90)
    rospy.loginfo ("12")


    Coordinate_System.travel_to(30,[-74.9,74.0])
    Coordinate_System.travel_to(30,[-74.6,107.9])
    Coordinate_System.travel_to(20,[-74.5,122.3])

    Coordinate_System.travel_to(20,[-74.0,162.4])
    #Coordinate_System.corner(15,18,90,[-38.6,195.0],0)
    Coordinate_System.pointToPointCorner(15,18,90,[-38.6,195.0],0)
    Coordinate_System.travel_to(20,[-15.2,194.3])
    rospy.loginfo ("13")


    #Coordinate_System.corner(15,8,0,[-3.1,175.0],-90)
    Coordinate_System.pointToPointCorner(15,8,0,[-3.1,175.0],-90)
    Coordinate_System.travel_to(20,[-3.8,150.2])
    Coordinate_System.travel_to(50,[-6,57.8])
    rospy.loginfo ("14")
    #"""

    # '''
    val=0
    while not rospy.is_shutdown():
        rospy.ROSInterruptException  # allow control+C to exit the program


        # val = val +1
        # rospy.loginfo("total run = " + str(val))
        
        Movement_Control.carControl(targetSpeed = 0,steerAngle = 0)



    

    





#################################################################################################################################################
try:
    # single time setup

    # start rosnode
    rospy.init_node('APC_Monash')
    rate = rospy.Rate(100) # publish data at 100Hz
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
