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
import shell_simulation_2.settings as settings 
import shell_simulation_2.ROS_Communication as ROS_Communication 
import shell_simulation_2.Movement_Control as Movement_Control 

#################################################################################################################################################
# import used libraries
import rospy
import math
import numpy as np
# used for picking the points, TODO remove later since we dont need
import random


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


def findGoalPoint(distancesToValidCoordinates):
    

    possible_coords = np.array([ [-171.60,4.00,0.00],   #P0
                        [-206.40,4.20,0.00],   #P1
                        [-255.90,0.20,0.00],   #P2
                        [-272.10,-43.90,0.00], #P3
                        [-205.50,-95.00,0.00], #P4
                        [-185.50,-142.40,0.00], #P5 #changed
                        [-151.10,-151.00,0.00],#P6
                        [-101.40,-154.70,0.00],#P7
                        [-47.80,-117.20,0.00], #P8
                        [-43.80,-56.80,0.00],  #P9
                        [-43.90,-17.10,0.00],  #P10
                        [3.00,-2.70,0.00],     #P11
                        [47.80,-1.80,0.00],    #P12
                        [89.00,-5.50,0.00],    #P13
                        [45.90,-84.90,0.00],   #P14
                        [31.30,19.30,0.00],    #P15
                        [36.30,67.20,0.00],    #P16
                        [38.60,155.10,0.00],   #P17
                        [74.00,190.20,0.00],   #P18 # changed
                        [154.10,177.30,0.00],  #P19
                        [189.20,52.80,0.00],   #P20
                        [174.40,-148.00,0.00], #P21
                        [10.20,-187.90,0.00],  #P22
                        [-145.80,-190.90,8.60],#P23
                        [-232.60,28.10,10.00], #P24
                        [-119.40,186.60,10.00],#P25
                        [84.70,144.10,0.00],   #P26
                        [148.10,112.20,0.00],  #P27
                        [151.40,15.20,0.00],   #P28
                        [124.70,1.90,0.00],    #P29
                        [96.20,-28.60,0.00],   #P30
                        [-9.50,-88.30,0.00],   #P31
                        [-83.20,-87.70,0.00],  #P32
                        [-124.30,-42.40,0.00], #P33
                        [-121.80,28.10,0.00],  #P34
                        [-124.40,106.30,0.00], #P35
                        [-80.20,133.30,0.00],  #P36
                        [-20.70,87.90,0.00],   #P37
                        [25.70,65.40,0.00],    #P38
                        [24.60,-30.70,0.00]    #P39
                        ])
    
    distance_to_possible_points = distances(possible_coords) # find the distances to the possible coordinates from the current position

    #print(distance_to_possible_points)
    # find where the distance to the goal is less than or equal to 3m, that will be a confirmed goal point
    possible_distances = distancesToValidCoordinates[np.where(distancesToValidCoordinates <= 3)]
    
    # if there are no possible goal points nearby then the possible_distances will be an empty array, whic if we use to index
    # will give us an error, so the following if statement is a simple way of checking the array is empty
    if possible_distances.shape[0] != 0:
        actual_point = distance_to_possible_points[abs(distance_to_possible_points - possible_distances) <= 3].min() # min because we may have more than one point
        print("Coords")
        return possible_coords[int(np.where(distance_to_possible_points == actual_point)[0]), :]

# Generates the random coordinates
def generate_random_coordinates():

    implemented_coords = [1,2,3,4,6,7,8]
    valid_coords = random.sample(implemented_coords, k=3)
    possible_coords = np.array([ [-171.60,4.00,0.00],   #P0
                        [-206.40,4.20,0.00],   #P1
                        [-255.90,0.20,0.00],   #P2
                        [-272.10,-43.90,0.00], #P3
                        [-205.50,-95.00,0.00], #P4
                        [-185.50,-142.40,0.00], #P5 #changed
                        [-151.10,-151.00,0.00],#P6
                        [-101.40,-154.70,0.00],#P7
                        [-47.80,-117.20,0.00], #P8
                        [-43.80,-56.80,0.00],  #P9
                        [-43.90,-17.10,0.00],  #P10
                        [3.00,-2.70,0.00],     #P11
                        [47.80,-1.80,0.00],    #P12
                        [89.00,-5.50,0.00],    #P13
                        [45.90,-84.90,0.00],   #P14
                        [31.30,19.30,0.00],    #P15
                        [36.30,67.20,0.00],    #P16
                        [38.60,155.10,0.00],   #P17
                        [74.00,190.20,0.00],   #P18 # changed
                        [154.10,177.30,0.00],  #P19
                        [189.20,52.80,0.00],   #P20
                        [174.40,-148.00,0.00], #P21
                        [10.20,-187.90,0.00],  #P22
                        [-145.80,-190.90,8.60],#P23
                        [-232.60,28.10,10.00], #P24
                        [-119.40,186.60,10.00],#P25
                        [84.70,144.10,0.00],   #P26
                        [148.10,112.20,0.00],  #P27
                        [151.40,15.20,0.00],   #P28
                        [124.70,1.90,0.00],    #P29
                        [96.20,-28.60,0.00],   #P30
                        [-9.50,-88.30,0.00],   #P31
                        [-83.20,-87.70,0.00],  #P32
                        [-124.30,-42.40,0.00], #P33
                        [-121.80,28.10,0.00],  #P34
                        [-124.40,106.30,0.00], #P35
                        [-80.20,133.30,0.00],  #P36
                        [-20.70,87.90,0.00],   #P37
                        [25.70,65.40,0.00],    #P38
                        [24.60,-30.70,0.00]    #P39
                        ])
    return np.array([possible_coords[coord, :] for coord in valid_coords])

def distances(coordinates):

    """Function to find the distances between the current position of the car and a set of coordinates,
     may it be the distances to all the coordinates or the distance to the valid coordinates
     Expect a ndarry for the coordinates"""

    current_pos = np.array([settings.car_coordinate_from_world])[:,:2]
    distances = np.sqrt(np.sum(np.square(coordinates[:,:2] - current_pos), axis=1))
    return distances
