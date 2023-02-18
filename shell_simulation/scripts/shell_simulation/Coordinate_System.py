#!/usr/bin/env python
'''
Coordinate_System (Level 3 code)
- uses Movement_Control.py

File purpose:
    - Translate coordinate goal and system to throttle and steering commands for the car
    - 

This function main purpose is to enables the car to goal to a designated corodinate using the following
- travel_to(setSpeed,goal_coord)       -- allowing the car to travel straight to a coordinate, and would end when within 2m from the goal
- corner


'''

#################################################################################################################################################
# import other prgramming files
import shell_simulation.settings as settings 
import shell_simulation.ROS_Communication as ROS_Communication 
import shell_simulation.Movement_Control as Movement_Control 

#################################################################################################################################################
# import used libraries
import rospy
import math
import numpy as np


#################################################################################################################################################

'''
Function Explanation :
    knowing the world coordinate of the car and the goal, 
    this function would use them to find the goal coordinate when the axis is placed on the car

    x axis would be the goal distance from the front of the car
    y axis would be the goal distance from the side of the car
    angle would be the angle of the goal from the front of the car
    
'''
def goal_position_from_car(goal_coord):
    '''
    function format
    since we mainly need x, y coordinate and orientation, we would use the following format for this function
        [x ,y , angle]
    
    it would be used in the followings 
    - car position with respect to the world
    - goal position with respect to the car before rotation
    - goal position with respect to the car
    '''
    car_from_world = [0,0,0]
    goal_from_car_before_rotation = [0,0,0]
    goal_from_car = [0,0,0]
    

    # 1) obtain current car coordinates from global variables
    # note that settings.car_direction_from_world is an absolute angle, (meaning that when you keep turining right, it is not limited to the -180 to 180 degree set by carla and thus go to 720, 1080 and so on)
    car_from_world = [  settings.car_coordinate_from_world[0], # x coordinate
                        settings.car_coordinate_from_world[1], # y coordinate
                        settings.car_direction_from_world[2]  # angle
                        ]
    
    # 2) calculate coordinate position from car
    goal_from_car_before_rotation[0]  =  goal_coord[0] - car_from_world [0] # x coordinate
    goal_from_car_before_rotation[1]  =  goal_coord[1] - car_from_world [1] # y coordinate


    # 3) find angle in the world coordinate of the goal from the car (before rotating axis)
    goal_from_car_before_rotation[2] = math.atan2(goal_from_car_before_rotation[1],goal_from_car_before_rotation[0] )  # value from -180 to 180



    '''
    coordinate transformation
    https://en.wikipedia.org/wiki/Rotation_of_axes#:~:text=If%20the%20curve%20(hyperbola%2C%20parabola,called%20a%20transformation%20of%20coordinates.
    x' = x cos(theta) + y sin(theta)
    y' = -x sin(theta) + y cos(theta)
    '''
    # 4) rotate axis and goal points around the car origin 
    goal_from_car[0] = goal_from_car_before_rotation[0] * math.cos((car_from_world[2]*math.pi/ 180 )) + goal_from_car_before_rotation[1] *math.sin((car_from_world[2]*math.pi / 180 ))
    goal_from_car[1] = -goal_from_car_before_rotation[0] * math.sin((car_from_world[2]*math.pi / 180 )) + goal_from_car_before_rotation[1]  *math.cos((car_from_world[2]*math.pi / 180 ))
    goal_from_car[2] =  (car_from_world[2]*math.pi) / 180  - goal_from_car_before_rotation[2] 
    

    # 5) filter system to ensure from car's prespective, that (-1 to -180 is turning left) & (1 to 180 is turning right) 
    goal_from_car[2] = (goal_from_car[2]*180)/math.pi
    while(goal_from_car[2]<-180):
        goal_from_car[2]= goal_from_car[2]+360

    while(goal_from_car[2]>180):
        goal_from_car[2] = goal_from_car[2]-360

     
    # 6) return goal's position and angle from car's prespective
    return goal_from_car  




#################################################################################################################################################

'''
Function Explanation :
    travel_to()
    -uses goal_position_from_car(goal_coord) function 

    it allows the car to move straight to a coordinate point 
    (does as aggressive steering as needed, and would drive straight to that point wihtout care for obstacles)
    
'''

def travel_to(setSpeed,goal_coord):
    
    # set up local variable to ensure output signal updates at 50Hz
    prev_time = settings.curr_time

    while not rospy.is_shutdown(): # ignore this loop when ctrl+c is already activated (to exit the program)
        rospy.ROSInterruptException  # allow ctrl+C to exit the program    

        # run at 100Hz to reduce computational power 
        # using non (search for arduino debounce if you're intrested in this method)
        if((settings.curr_time - prev_time) > 0.01):  
            prev_time = settings.curr_time

            # 1) run function to obtain goal coordinates & angle from current car position
            goal_coord_from_car = goal_position_from_car(goal_coord)
            

            # 2) obtain goal distance from car (using pythagoras theorem)
            diff_goal = math.sqrt(math.pow(goal_coord_from_car[0], 2) + math.pow(goal_coord_from_car[1], 2))


            # 3) send commands to carla to control the car
            Movement_Control.carControl(targetSpeed = setSpeed,steerAngle= goal_coord_from_car[2])


            # stop the loop when the goal is within 0.6m of the car
            if diff_goal<=1:
                break
   
################################################################################################################################################# 
'''
Function Explanation :
    intersect_Point()
    - form two lines using their angle and coordinate
    - it then find the intersection point of the two lines
    - return the coordinate of the intersection point

    - angle is in degrees
'''
def intersect_Point(start_Pos,start_Angle,end_Pos,end_Angle):
    # convert angles from degrees to radians
    start_Angle = start_Angle*math.pi/180
    end_Angle = end_Angle*math.pi/180

    # 1) the equation for finding the intersection point can be found in the documentation

    # 2) solve given simultaneous equation using numpy libraries (ENG1005)
    #    to get the unknown variables 
    A = np.array([
        [math.cos(start_Angle), math.cos(end_Angle)],
        [math.sin(start_Angle), math.sin(end_Angle)]
        ])
    B = np.array([end_Pos[0]-start_Pos[0],end_Pos[1]-start_Pos[1]])
    C = np.linalg.solve(A,B)

    # 3) use varaibles found in step 2 to find the coordinate    
    Coordinate_Point = [start_Pos[0]+C[0]*math.cos(start_Angle)  ,  start_Pos[1]+C[0]*math.sin(start_Angle)]

    return Coordinate_Point


'''
Function Explanation :
    corner()
    - allows the car to turn at the right place, given the required parameters
'''
def corner(speed,turnRadius,start_Angle,end_Pos,end_Angle):
    start_Pos = [settings.car_coordinate_from_world[0],settings.car_coordinate_from_world[1]]
    # start_Angle = settings.car_direction_from_world[2]
    '''
    # use for troubleshooting
    rospy.loginfo("start_Angle: " + str(start_Angle ))
    rospy.loginfo("end_Angle: " + str(end_Angle))
    rospy.loginfo("  ")
    rospy.loginfo("start: " + str(start_Pos))
    rospy.loginfo("end: " + str(end_Pos))
    '''

    # obtain intersection Point 
    intersectionPoint = intersect_Point(start_Pos,start_Angle,end_Pos,end_Angle)


    # convert angles to radians
    start_Angle = start_Angle*math.pi/180
    end_Angle = end_Angle*math.pi/180
    
    
    # solving for distance of circle hitting the line from intersection point (more info in documentation)
    turn_angle = -abs(end_Angle+start_Angle) /2    
    offset = abs(turnRadius/math.tan(turn_angle))
    

    
    # apply offset to turn using starting angle 
    turnCoord = [   intersectionPoint[0]-offset*math.cos(start_Angle),
                    intersectionPoint[1]-offset*math.sin(start_Angle)]

    '''
    # use for troubleshooting
    rospy.loginfo("intersectionPoint: " + str(intersectionPoint))
    rospy.loginfo("turn_angle: " + str(turn_angle*180/math.pi))
    rospy.loginfo("offset: " + str(offset))
    rospy.loginfo("TurnCoord " + str(turnCoord))
    '''

    # before turning - drive to the calculated coordinate to start turning
    travel_to(speed, turnCoord)    

    # while turning - turn at constant angle
    while not rospy.is_shutdown():
        rospy.ROSInterruptException  # allow control+C to exit the program

        # find angle to turn (in documentation or can be found here https://autoinfome.blogspot.com/p/turning-radius-turning-circle.html)     
        angle = -math.asin(settings.wheelBase/turnRadius)
        
        # invert angle when turning right
        if(goal_position_from_car(end_Pos)[2]>0):
            angle=-angle

        # send turning command to Carla
        Movement_Control.carControl(targetSpeed = speed,steerAngle= angle *180/math.pi)

        # exit when turned most of the way
        if abs(goal_position_from_car(end_Pos)[2] - end_Angle) <= 15:
            break
    
    # after turning - drive to final coordinate 
    travel_to(speed, end_Pos)
