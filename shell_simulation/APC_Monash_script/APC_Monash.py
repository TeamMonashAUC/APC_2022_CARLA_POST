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

#################################################################################################################################################
# import libraries
import rospy
import math

#################################################################################################################################################

def main():
    
    travel_to(15, [-77.9,-13])
    corner(15,[-110.4,3.4], 10,180)


    travel_to(15, [-134,3.6])
    rospy.loginfo("1")
    corner(15,[-145.4,25.2], 10,90)

    # travel_to(15, [-134,3.6])
    # corner(15,[-149.4,-15.5], 15,270)



    # corner(15,[-44.2,-0.8], 10,0)
    
    # goal_map = PoseStamped()
    # goal_map.header = settings.odom
    # rospy.loginfo(settings.odom)
    # goal_map.pose.position.x = -77.8
    # goal_map.pose.position.y = 50

    # output = tf2_buffer.transform(goal_map,"ego_vehicle")
    
   
    # travel_to([-74.8,-13.8])
    # travel_to([-71.5,-3.2])
    # travel_to([-64.2,-0.8])
    # travel_to([-41.4,-1.2])
    # travel_to([-29.2,-3.7])
    # travel_to([-21.4,-11.5])
    # travel_to(20,[0,0])

    # travel_to(80,[-71.6,150])
    # travel_to(20,[-71.6,194])
    # rospy.loginfo ("1")

    # travel_to(80,[120,194])
    # travel_to(20,[150.1,194])
    # rospy.loginfo ("2")

    # travel_to(80,[150,30.4])
    # travel_to(20,[150,8.4])
    # rospy.loginfo ("3")

    # travel_to(80,[40,8])
    # travel_to(20,[24,8])
    # rospy.loginfo ("4")

    # travel_to(40,[-77.8,11])
    # rospy.loginfo ("1")
    # travel_to(20,[-77.8,50])
    # rospy.loginfo ("2")
    # travel_to(40,[-77.8,120])
    # rospy.loginfo ("3")
    # travel_to(20,[-77.8,175])
    # rospy.loginfo ("4")

    travel_to(15, [-77.9,-13])
    # corner(15,[-44.2,-0.8], 10,0)
    corner(15,[-110.4,3.4], 10,180)



    # travel_to(20,[-110.4,3.4])
    # to do
    # 1) adapt calculations for left turn as well (apply from car coordinate instead of world)
    # 2) adapt corner to turn into any angled road (45 degree road)
    # 3) possible improvement to coordinate system in travel_to function



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

    rospy.loginfo([intersectPos[0], intersectPos[1]-turnRadius])
    travel_to(speed, [intersectPos[0], intersectPos[1]-turnRadius]  )

    # turnRightDir = 0 # 0 for turn left, 1 for turning right 
    if target_angle < settings.car_direction_from_world[2]:
        turnRightDir = 1
    else :
        turnRightDir = 0

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

def travel_to(setSpeed,goal_coord):
    
    # diff_goal = math.sqrt(math.pow(goal_coord_from_car[0], 2) + math.pow(goal_coord_from_car[1], 2))
    # rospy.loginfo (diff_goal)
    prev_time = settings.curr_time

    while not rospy.is_shutdown():
        # rospy.loginfo(settings.car_direction_from_world[2])

        if((settings.curr_time - prev_time) > 0.02):  # run at 50Hz to reduce computational power
            goal_coord_from_car = goal_distance_from_car(target_coord = goal_coord, from_coord = [settings.car_coordinate_from_world[0],settings.car_coordinate_from_world[1]])

            # rospy.loginfo(goal_coord_from_car)

            diff_goal = math.sqrt(math.pow(goal_coord_from_car[0], 2) + math.pow(goal_coord_from_car[1], 2))
            # diff_goal = 1
            
            # if(goal_coord_from_car[0]>=0):
            # ori_angle = -(math.atan2(goal_coord_from_car[1],goal_coord_from_car[0])*180)/math.pi
            # angle = ori_angle
            # if(settings.current_quadrant ==3):
            #     if(angle<0):
            #         angle = 360-ori_angle
            # elif(settings.current_quadrant==2):
            #     if(angle>0):
            #         angle = ori_angle-360 
            # rospy.loginfo(f"{angle} {ori_angle}")
            # else:
            #     angle = -(math.atan2(goal_coord_from_car[1],-goal_coord_from_car[0])*180)/math.pi

            # if(settings.car_direction_from_world[2])
            # if(angle >)
            

            # rospy.loginfo(f"{angle}")



            prev_time = settings.curr_time
            # rospy.loginfo(goal_coord_from_car[2])
            Movement_Control.carControl(targetSpeed = setSpeed,steerAngle= goal_coord_from_car[2])

            rospy.ROSInterruptException  # allow control+C to exit the program
            if diff_goal<=0.5:
                break
    
    
    




# obtain coordinate target_coordinate with respects to from_coordinate
# example, goal coordinate with respect to car (from car's perspective)
def goal_distance_from_car(target_coord, from_coord):
    result_coord = [0,0,0]
    result_coord[0] = target_coord[0]-from_coord[0] # x coordinate
    result_coord[1] = target_coord[1]-from_coord[1] # y coordinate

    world_angle = (math.atan2(result_coord[1],result_coord[0])*180)/math.pi  # value from -180 to 180
    goal_from_car_angle = settings.car_direction_from_world[2]-world_angle 
    
    while(goal_from_car_angle<-180):
        goal_from_car_angle = goal_from_car_angle+360

    while(goal_from_car_angle>180):
        goal_from_car_angle = goal_from_car_angle-360

    result_coord[2] = goal_from_car_angle
    
    
    # rospy.loginfo(f"{result_coord} {world_angle}")
     
    return result_coord

#################################################################################################################################################
try:
    # single time setup

    # start rosnode
    rospy.init_node('APC_Monash')
    rate = rospy.Rate(50) # publish data at 100Hz
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
