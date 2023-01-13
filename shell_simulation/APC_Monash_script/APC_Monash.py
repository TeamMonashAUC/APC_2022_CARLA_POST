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
    
    # '''
    # test straight line function 1
    travel_to(80,[-71.6,150])
    travel_to(20,[-71.6,194])
    rospy.loginfo ("1")

    travel_to(80,[120,194])
    travel_to(20,[150.1,194])
    rospy.loginfo ("2")

    travel_to(80,[150,30.4])
    travel_to(20,[150,8.4])
    rospy.loginfo ("3")

    travel_to(80,[40,8])
    travel_to(20,[24,8])
    rospy.loginfo ("4")
    # '''

    '''
    # test straight line function 2
    travel_to(20,[-86.6,-97.2])
    travel_to(20,[-69.3,-184.1])
    rospy.loginfo ("1")

    travel_to(80,[177.7,-195.5])
    travel_to(20,[240.1,-135.7])
    travel_to(20,[240.1,-68.8])
    rospy.loginfo ("2")

    travel_to(20,[185,-58.6])
    # travel_to(20,[150,8.4])
    travel_to(20,[157.6,-121.2])
    rospy.loginfo ("3")

    travel_to(80,[-2,-130.2])
    # travel_to(20,[24,8])
    rospy.loginfo ("4")
    '''


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
    
    # set up local variable to ensure 50Hz signal
    prev_time = settings.curr_time

    while not rospy.is_shutdown(): # ignore this loop when ctrl+c is already activated (to exit the program)
        rospy.ROSInterruptException  # allow ctrl+C to exit the program    

        # run at 50Hz to reduce computational power 
        # using non (search for arduino debounce if you're intrested in this method)
        if((settings.curr_time - prev_time) > 0.02):  
            prev_time = settings.curr_time

            # 1) run function to obtain goal coordinates & angle from current car position
            goal_coord_from_car = goal_position_from_car(goal_coord)
            

            # 2) obtain goal distance from car (using pythagoras theorem)
            diff_goal = math.sqrt(math.pow(goal_coord_from_car[0], 2) + math.pow(goal_coord_from_car[1], 2))


            # 3) send commands to carla to control the car
            Movement_Control.carControl(targetSpeed = setSpeed,steerAngle= goal_coord_from_car[2])


            # stop the loop when the goal is within 2m of the car
            if diff_goal<=2:
                break
    
    
    




# obtain coordinate target_coordinate with respects to from_coordinate
# example, goal coordinate with respect to car (from car's perspective)
def goal_position_from_car(goal_coord):
    '''
    function format
    since we mainly need x, y coordinate and orientation, we would use the following format for this function
        [x ,y , angle]
    
    it would be used in the followings 
    - car position with respect to the world
    - goal position with respect to the car
    '''
    car_from_world = [0,0,0]
    goal_from_car = [0,0,0]
    

    # 1) obtain current car coordinates from global variables
    # note that settings.car_direction_from_world is an absolute angle, (meaning that when you keep turining right, it is not limited to the -180 to 180 degree set by carla and thus go to 720, 1080 and so on)
    car_from_world = [  settings.car_coordinate_from_world[0], # x coordinate
                        settings.car_coordinate_from_world[1], # y coordinate
                        settings.car_direction_from_world[2]  # angle
                        ]
    
    # 2) find x and y coordinate of the goal from the car
    goal_from_car[0] = goal_coord[0] - car_from_world [0] # x coordinate
    goal_from_car[1] = goal_coord[1] - car_from_world [1] # y coordinate


    # 3) find angle in the world coordinate of the goal from the car (without taking car angle in account)
    car_and_goal_angle_diff_in_world = (math.atan2(goal_from_car[1],goal_from_car[0])*180)/math.pi  # value from -180 to 180


    # 4) offset angle by car current angle 
    goal_from_car[2] =  car_from_world[2]  - car_and_goal_angle_diff_in_world 

    
    # 5) filter system to ensure from car's prespective, that (-1 to -180 is turning left) & (1 to 180 is turning right) 
    while(goal_from_car[2]<-180):
        goal_from_car[2]= goal_from_car[2]+360

    while(goal_from_car[2]>180):
        goal_from_car[2] = goal_from_car[2]-360

    # goal_from_car[2] = goal_from_car[2]
    
     
    # 6) return goal's position and angle from car's prespective
    return goal_from_car  



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
