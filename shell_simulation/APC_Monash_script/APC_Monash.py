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
    # first corner (working)
    Coordinate_System.travel_to(15, [-77.9, -15])
    corner(15,5,[-44.2, -1],-90)
    # '''
    
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

def corner(speed,turnRadius,end_Pos,end_Angle):
    # start_Pos = [0,0]
    # start_Pos = [-74.754638671875, 102.29220581054688]
    # start_Angle = 90 -90
    # end_Angle = end_Angle
    
    # start_Angle = 0
    # start_Pos = [-74.754638671875, 102.29220581054688]
    # start_Angle = 88.35488891601564
    start_Pos = [settings.car_coordinate_from_world[0],settings.car_coordinate_from_world[1]]
    start_Angle = settings.car_direction_from_world[2]+end_Angle

    # rospy.loginfo(goal_position_from_car(end_Pos))

    rospy.loginfo("start: " + str(start_Pos))
    rospy.loginfo("end: " + str(end_Pos))
    rospy.loginfo("start_Angle: " + str(start_Angle ))
    rospy.loginfo("end_Angle: " + str(end_Angle))
    # rospy.loginfo("  ")

    start_Angle = start_Angle*math.pi/180
    end_Angle = end_Angle*math.pi/180


    # solve simultaneous equation

    A = np.array([
        [math.sin(start_Angle), -math.sin(end_Angle)],
        [math.cos(start_Angle), -math.cos(end_Angle)]
        ])
    B = np.array([end_Pos[0]-start_Pos[0],end_Pos[1]-start_Pos[1]])
    C = np.linalg.solve(A,B)
    # rospy.loginfo("A" + str(A))
    # rospy.loginfo("B" + str(B))
    # rospy.loginfo("C" + str(C))
    # rospy.loginfo("  ")

    intersectionPoint = [end_Pos[0]+C[1]*math.sin(end_Angle),
                end_Pos[1]+C[1]*math.cos(end_Angle)]

    # P3_coord = [end_Pos[0] + C[1]*math.sin(end_Angle), end_Pos[1] + C[1]*math.cos(end_Angle) ]

    rospy.loginfo("intersectionPoint" + str(intersectionPoint))
    # rospy.loginfo("  ")

    angle_diff = end_Angle - start_Angle
    d = turnRadius*math.tan(angle_diff/2 )+settings.wheelBase

    # turnCoord = [   P3_coord[0]-d*math.sin((-start_Angle*math.pi)/360)+start_Pos[0],
    #                 -P3_coord[1]-d*math.cos((start_Angle*math.pi)/360)+start_Pos[1]]
    turnCoord = [   intersectionPoint[0]+d*math.sin((start_Angle*math.pi)/360),
                    intersectionPoint[1]+d*math.cos((start_Angle*math.pi)/360)]
    # rospy.loginfo("math" + str(P3_coord)+" "+str(d*math.cos((start_Angle*math.pi)/360))+" "+str(start_Pos[1]))
    rospy.loginfo("TurnCoord" + str(turnCoord))
    rospy.loginfo("  ")
    rospy.loginfo("  ")



    # rospy.loginfo(goal_position_from_car(end_Pos))
    Coordinate_System.travel_to(speed, turnCoord)
    '''
    RightDir = 0 # 0 ?
    # turnfor turn left, 1 for turning right 
    # if end_Angle < settings.car_direction_from_world[2]:
    #     turnRightDir = 0
    # else :
    #     turnRightDir = 1
    '''
    while not rospy.is_shutdown():
        angle = math.asin(settings.wheelBase/turnRadius)
        rospy.ROSInterruptException  # allow control+C to exit the program

        # rospy.loginfo(angle *180/math.pi)
        Movement_Control.carControl(targetSpeed = speed,steerAngle= angle *180/math.pi)
        if Coordinate_System.goal_position_from_car(end_Pos)[2] <= end_Angle+15:
            break

    Coordinate_System.travel_to(speed, end_Pos)
    '''
    #1 get startPos,endPos and radius 
    startPos = [settings.car_coordinate_from_world[0],settings.car_coordinate_from_world[1]]
    # startAngle = settings.car_direction_from_world[2]
    
    # rospy.loginfo(startPos)
    # 2 calculate intersection point
    
    intersectPos = [startPos[1],endPos[0]]
    # rospy.loginfo(intersectPos)
    
    #3 find start curning coordinate

    rospy.loginfo([intersectPos[0], intersectPos[1]+turnRadius])  
    travel_to(speed, [intersectPos[0], intersectPos[1]+turnRadius]  )
    RightDir = 0 # 0 
    # turnfor turn left, 1 for turning right 
    if target_angle < settings.car_direction_from_world[2]:
        turnRightDir = 0
    else :
        turnRightDir = 1

    while not rospy.is_shutdown():
        angle = math.asin(settings.wheelBase/turnRadius)
        rospy.ROSInterruptException  # allow control+C to exit the program

        # Movement_Control.carControl(targetSpeed = speed,steerAngle= angle *180/math.pi)
        # if settings.car_direction_from_world[2] <target_angle:
        #     break
        if(turnRightDir):
            Movement_Control.carControl(targetSpeed = speed,steerAngle= angle *180/math.pi)
            if settings.car_direction_from_world[2] <target_angle:
                break
        else:
            Movement_Control.carControl(targetSpeed = speed,steerAngle= -angle *180/math.pi)
            if settings.car_direction_from_world[2] >target_angle:
                break
        

    travel_to(speed,endPos)
    '''

    





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
