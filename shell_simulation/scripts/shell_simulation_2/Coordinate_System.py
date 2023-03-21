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
    prev_time_coord = settings.curr_time

    while not rospy.is_shutdown(): # ignore this loop when ctrl+c is already activated (to exit the program)
        rospy.ROSInterruptException  # allow ctrl+C to exit the program    

        # run at 20Hz to reduce computational power 
        # using non (search for arduino debounce if you're intrested in this method)
        if((settings.curr_time - prev_time) > 0.05):  
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

        # obtain coordinates of goal points every 3 seconds
        # if((settings.curr_time - prev_time_coord) > 3):
        #     prev_time_coord = settings.curr_time
        #     update_Coord()

    
   
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
    turn_angle = abs(end_Angle+start_Angle)    
    half_turn_angle = turn_angle /2    
    offset = abs(turnRadius/math.tan(half_turn_angle))
    
    
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

def corner2(speed,turnRadius,start_Angle,end_Pos,end_Angle):
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
    turn_angle = abs(end_Angle+start_Angle)    
    half_turn_angle = turn_angle /2    
    offset = abs(turnRadius/math.tan(half_turn_angle))
    
    
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
    travel_to2(speed, turnCoord)    

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
    travel_to2(speed, end_Pos)

def pointToPointCorner(speed,turnRadius,start_Angle,end_Pos,end_Angle, number_of_points=10):
    """
    Function: pointToPointCorner()
    Paramters:  speed - The speed which the car has to turn
                turnRadius - The radius of the imaginary circle we trace out when turning
                start_Angle - The angle which we start at***
                end_Angle - The angle which we end at***
                number_of_points - The number of points on the imaginary turning circle we generate(more points smoother curve**)
    
    *** Due to angle generation issues do not use -180, use +180, since they mean the same thing but it makes the angle generation easier.
    ** Small number of points will suffice, sometimes have the issue of missing a point if too many points are used.(need to do more testing)

    This function works very similarly to the corner() function, where we find the intersection point, then offset that to find the turning point.
    However in this function, we use the turning radius and resolve it to find the center of the circle we need to follow to make the turn.
    Once the center is found, we find the angles required to take the curved path, then using the parameteric equation of the circle:
        x = r*cos(theta) + x0
        y = r*sin(theta) + y0
    We generate the arc by giving the valid range of theta, the travelling to the corresponding x and y.

    # TODO currently the turning works for 90 degree turns, havent tested different angle turns since fixing the angle generation issue.
    """

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


    arc_center = [0, 0]
    
    # Using the angle seen by the car to the final position to identify if its a left or right turn,
    # if the angle is negative its a left turn, otherwise its a right turn.

    # If it is a left turn then the angles have to sweep counter clockwise
    # If it is a right turn then the angles have to sweep clockwise
    # There are two edge cases, which are placed in the angle finding
    # These two edge cases exist as at 180 if we add 90, we reach 270 which is what we want but when we command python to generate (270:0) angles 
    # it generates as 270 260 250... 30 20 0, what we actually want is 270 280 290.. 250 360/0, so we dont allow that to happen, similar issue comes
    # comes on the negative side as well.

    # sign is used later to find the center of the arc, since according to the direction of rotation the center could be to the left
    # or the right of the car.
    if goal_position_from_car(end_Pos)[2] >= 0:
        sign = 1
        rospy.loginfo(f"Right Turn")
        angles = np.linspace(start_Angle + np.pi/2, end_Angle + np.pi/2 if end_Angle + np.pi/2 <= np.pi else np.pi/2 - end_Angle, number_of_points)
        
    else:
        sign = -1
        angles = np.linspace(start_Angle - np.pi/2, end_Angle - np.pi/2 if end_Angle - np.pi/2 > -np.pi else abs(end_Angle - np.pi/2), number_of_points)
        rospy.loginfo("Left Turn")
        

        
    # Same way we apply the offset we find the center of rotation using resolving.
    # this case its more complicated since according to the direction of rotation, the center of rotation can be on the left or the right
    arc_center[0] = turnCoord[0] + (sign)*turnRadius*np.sin(start_Angle)
    arc_center[1] = turnCoord[1] - (sign)*turnRadius*np.cos(start_Angle)
    

    #rospy.loginfo(f"The tan of the angle, {np.tan(start_Angle)}, the arctan of the other angle {np.arctan(-1/np.tan(start_Angle))}")

    #rospy.loginfo(f"Actual arc center: {arc_center}")

    #rospy.loginfo(f"The center of the circle is x = {arc_center[0]}, y = {arc_center[1]}")
    
    #rospy.loginfo(f"The angles {angles*180/np.pi}")
    
    # Generate the x and y arcs as a parameter of the angle
    # The parametric equation of a circle
    arc_x = arc_center[0] + turnRadius*np.cos(angles)
    arc_y = arc_center[1] + turnRadius*np.sin(angles)

    # Uncomment to find if the travel arc is what we desire
    #rospy.loginfo(f"The x coords are {arc_x}")
    #rospy.loginfo(f"The y coords are {arc_y}")



    # before turning - drive to the calculated coordinate to start turning
    travel_to(speed, turnCoord)
    
    # suspecting that this might be the issue, alot of travel to, probably slows down when it gets close to the point.
    # We are following the circular arc that was previously generated
    for i in range(len(arc_x)):
        #rospy.loginfo(f"{arc_x[i]}, {arc_y[i]}")
        travel_to(speed, (arc_x[i], arc_y[i]))
    
    travel_to(speed, end_Pos)

'''
def findGoalPoint(distancesToValidCoordinates, goalPredict):
    

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
    #possible_distances = distancesToValidCoordinates[distancesToValidCoordinates <= 3]
    
    # if there are no possible goal points nearby then the possible_distances will be an empty array, whic if we use to index
    # will give us an error, so the following if statement is a simple way of checking the array is empty
    #if possible_distances.shape[0] != 0:
    #actual_point = distance_to_possible_points[abs(distance_to_possible_points - distance_to_possible_points) <= 0.1].min() # min because we may have more than one point

    correct_distances = distance_to_possible_points[np.isin(np.round(distance_to_possible_points, 0), np.round(distancesToValidCoordinates, 0))]
    validPoints = possible_coords[np.isin(distance_to_possible_points, correct_distances)]
    
    goalPredict.append(validPoints)
    #goalPredict.append(possible_coords[int(np.where(distance_to_possible_points == actual_point)[0]), :])
    #return possible_coords[int(np.where(distance_to_possible_points == actual_point)[0]), :]
'''
# Generates the random coordinates
def generate_random_coordinates():

    implemented_coords = [1,2,3,4,6,7,8]
    # implemented_coords = [0,1,2,3,4,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31,32,33,34,35,36,37,38,39]
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

def distances2D(coordinates):

    """Function to find the distances between the current position of the car and a set of coordinates,
     may it be the distances to all the coordinates or the distance to the valid coordinates
     Expect a ndarry for the coordinates"""

    current_pos = np.array([settings.car_coordinate_from_world])[:,:2]
    distances = np.sqrt(np.sum(np.square(coordinates[:,:2] - current_pos), axis=1))
    return distances


def distances3D(coordinates):

    """Function to find the distances between the current position of the car and a set of coordinates,
     may it be the distances to all the coordinates or the distance to the valid coordinates
     Expect a ndarry for the coordinates"""

    current_pos = np.array([settings.car_coordinate_from_world])
    distances = np.sqrt(np.sum(np.square(coordinates - current_pos), axis=1))
    return distances


def travel_to2(setSpeed,goal_coord):

    # set up local variable to ensure output signal updates at 50Hz
    prev_time = settings.curr_time
    prev_time_coord = settings.curr_time
    gradual_increase = 1.1 #change this to affect acceleration
    while not rospy.is_shutdown(): # ignore this loop when ctrl+c is already activated (to exit the program)
        rospy.ROSInterruptException  # allow ctrl+C to exit the program    
        starting_speed = settings.currentCarSpeed
        # run at 20Hz to reduce computational power 
        # using non (search for arduino debounce if you're intrested in this method)
        if((settings.curr_time - prev_time) > 0.05):  
            prev_time = settings.curr_time

            # 1) run function to obtain goal coordinates & angle from current car position
            goal_coord_from_car = goal_position_from_car(goal_coord)

            starting_speed = settings.currentCarSpeed
            # 2) obtain goal distance from car (using pythagoras theorem)
            diff_goal = math.sqrt(math.pow(goal_coord_from_car[0], 2) + math.pow(goal_coord_from_car[1], 2))
            

            # 3) send commands to carla to control the car
            #attempting to increase speed gradually
            # print("Set speed: ", setSpeed)
            # print("Starting speed: ", starting_speed)
            # print("")
            if(starting_speed < (setSpeed) -gradual_increase):
                starting_speed +=gradual_increase
                # print("increasing speed")
                Movement_Control.carControl(targetSpeed = starting_speed,steerAngle= goal_coord_from_car[2]) 

            elif(starting_speed > (setSpeed)+gradual_increase):
                starting_speed -=gradual_increase
                # print("decreasing speed")
                Movement_Control.carControl(targetSpeed = starting_speed,steerAngle= goal_coord_from_car[2])

            else:
                Movement_Control.carControl(targetSpeed = setSpeed,steerAngle= goal_coord_from_car[2])




            # stop the loop when the goal is within 0.6m of the car
            if diff_goal<=1:
                break

    # obtain coordinates of goal points every 3 seconds
    # if((settings.curr_time - prev_time_coord) > 3):
    #     prev_time_coord = settings.curr_time
    #     update_Coord() 
def findGoalPointRobust2D(distancesToValidCoordinates):
    goal_predict = []

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
    
    distance_to_possible_points = distances2D(possible_coords) # find the distances to the possible coordinates from the current position
    
    # Assuming that the distance supplied is a normal python list
    for distance in distancesToValidCoordinates:
        possible_distance = distance_to_possible_points[abs(distance_to_possible_points - distance) <= 1]
        if possible_distance.shape[0] != 0:
            goal_predict.append(possible_coords[np.where(distance_to_possible_points == possible_distance)[0], :].tolist())
    return goal_predict
    #print(distance_to_possible_points)
    # find where the distance to the goal is less than or equal to 3m, that will be a confirmed goal point
    #possible_distances = distancesToValidCoordinates[distancesToValidCoordinates <= 3]
    
    # if there are no possible goal points nearby then the possible_distances will be an empty array, whic if we use to index
    # will give us an error, so the following if statement is a simple way of checking the array is empty
    #if possible_distances.shape[0] != 0:
    #actual_point = distance_to_possible_points[abs(distance_to_possible_points - distance_to_possible_points) <= 0.1].min() # min because we may have more than one point

    # correct_distances = distance_to_possible_points[np.isin(np.round(distance_to_possible_points, 0), np.round(distancesToValidCoordinates, 0))]
    # validPoints = possible_coords[np.isin(distance_to_possible_points, correct_distances)]
    
    # goalPredict.append(validPoints)

def findGoalPointRobust3D(distancesToValidCoordinates):
    goal_predict = []

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
    
    distance_to_possible_points = distances3D(possible_coords) # find the distances to the possible coordinates from the current position
    
    # Assuming that the distance supplied is a normal python list
    for distance in distancesToValidCoordinates:
        possible_distance = distance_to_possible_points[abs(distance_to_possible_points - distance) <= 1]
        if possible_distance.shape[0] != 0:
            goal_predict.append(possible_coords[np.where(distance_to_possible_points == possible_distance)[0], :].tolist())
    return goal_predict


coord_2d = [[]]
coord_3d = [[]]
def update_Coord():
    for value in findGoalPointRobust2D(settings.coord_distance):
        if value not in coord_2d:
            coord_2d.append(value)
    for value in findGoalPointRobust3D(settings.coord_distance):
        if value not in coord_3d:
            coord_3d.append(value)
    
    # print(f"The goal 2Dcoordinates {coord_2d}")
    # print(f"The goal 3Dcoordinates {coord_3d}")

    settings.coord_2d = coord_2d
    settings.coord_3d = coord_3d
   