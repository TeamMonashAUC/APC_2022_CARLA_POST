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
import settings
import ROS_Communication
import Movement_Control

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


            # stop the loop when the goal is within 2m of the car
            if diff_goal<=0.2:
                break
   
################################################################################################################################################# 
def intersect_Point(start_Pos,start_Angle,end_Pos,end_Angle):

    start_Angle = start_Angle*math.pi/180
    end_Angle = end_Angle*math.pi/180

    diff_Angle = start_Angle-end_Angle
    # rospy.loginfo(str(diff_Angle*180/math.pi))
    # solve simultaneous equation
    A = np.array([
        [math.sin(start_Angle), -math.sin(end_Angle)],
        [math.cos(start_Angle), -math.cos(end_Angle)]
        ])
    B = np.array([end_Pos[0]-start_Pos[0],end_Pos[1]-start_Pos[1]])
    C = np.linalg.solve(A,B)
    Point =[end_Pos[0]-C[0]*math.sin(start_Angle)  ,  end_Pos[1]-C[0]*math.cos(start_Angle)]
    # Point =[start_Pos[0]+C[1]*math.cos(start_Angle)  ,  start_Pos[1]+C[1]*math.sin(start_Angle)]
    # Point =[end_Pos[0]+C[0]*math.cos(end_Angle)  ,  start_Pos[1]-C[1]*math.sin(start_Angle)]


    return Point



def corner(speed,turnRadius,start_Angle,end_Pos,end_Angle):
    start_Pos = [settings.car_coordinate_from_world[0],settings.car_coordinate_from_world[1]]
    # start_Angle = settings.car_direction_from_world[2]


    rospy.loginfo("start_Angle: " + str(start_Angle ))
    rospy.loginfo("end_Angle: " + str(end_Angle))
    rospy.loginfo("  ")
    rospy.loginfo("start: " + str(start_Pos))
    rospy.loginfo("end: " + str(end_Pos))
    
    
    
    # solve simultaneous equation
    # A = np.array([
    #     [math.sin(start_Angle), -math.sin(end_Angle)],
    #     [math.cos(start_Angle), -math.cos(end_Angle)]
    #     ])
    # B = np.array([end_Pos[0]-start_Pos[0],end_Pos[1]-start_Pos[1]])
    # C = np.linalg.solve(A,B)

    # intersectionPoint = [start_Pos[0]-C[0]*math.cos(start_Angle)  ,  start_Pos[1]-C[0]*math.sin(start_Angle)]
    # intersectionPoint = [end_Pos[0]-C[0]*math.sin(start_Angle)  ,  end_Pos[1]-C[0]*math.cos(start_Angle)]
    intersectionPoint = intersect_Point(start_Pos,start_Angle,end_Pos,end_Angle)
    # rospy.loginfo("C0 sin start angle: " + str(-C[0]*math.sin(start_Angle)))
    # rospy.loginfo("C0 cos start angle: " + str(-C[0]*math.cos(start_Angle)))
    rospy.loginfo("intersectionPoint: " + str(intersectionPoint))

    start_Angle = start_Angle*math.pi/180
    end_Angle = end_Angle*math.pi/180
    # rospy.loginfo("C0: " + str(C[0]))
    # rospy.loginfo("C1: " + str(C[1]))

    # apply offset to turn in using starting angle
    turnCoord = [   intersectionPoint[0]-turnRadius*math.cos(start_Angle),
                    intersectionPoint[1]-turnRadius*math.sin(start_Angle)]


    # rospy.loginfo("offset x: " + str(-turnRadius*math.cos(start_Angle)))
    # rospy.loginfo("offset y: " + str(-turnRadius*math.sin(start_Angle)))
    # rospy.loginfo("TurnCoord" + str(turnCoord))
    # rospy.loginfo("  ")
    # rospy.loginfo("  ")

    travel_to(speed, turnCoord)
    
    while not rospy.is_shutdown():
        rospy.ROSInterruptException  # allow control+C to exit the program

        angle = -math.asin(settings.wheelBase/turnRadius)
        Movement_Control.carControl(targetSpeed = speed,steerAngle= angle *180/math.pi)
        # rospy.loginfo(angle *180/math.pi)

        if abs(goal_position_from_car(end_Pos)[2]- end_Angle) <= 15:
            break

    rospy.loginfo("finalTurn")
    travel_to(speed, end_Pos)
