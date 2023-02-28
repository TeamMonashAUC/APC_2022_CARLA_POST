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


################################################
# Camera OpenCV2
import carla
import time
import cv2
import numpy as np

im_height = 1080
im_width = 1920


client = carla.Client("localhost",2000)
client.set_timeout(2.0)
world = client.get_world
# Find the blueprint of the sensor.
bp= world.get_blueprint_library().find('sensor.camera.rgb')
# Modify the attributes of the blueprint to set image resolution and field of view.
bp.set_attribute('image_size_x', f"{im_width}")
bp.set_attribute('image_size_y', f"{im_height}")
bp.set_attribute('fov', '110')
# Provide the position of the sensor relative to the vehicle.
cam_position = carla.Transform(carla.Location(x=0.8, z=1.7))
# Tell the world to spawn the sensor, don't forget to attach it to your vehicle actor.
sensor = world.spawn_actor(bp, cam_position, attach_to=controls)
# Subscribe to the sensor stream by providing a callback function, this function is
# called each time a new image is generated by the sensor.
sensor.listen(lambda data: process_img(data))

def process_img(image):
    i = np.array(image.raw_data)
    i2 = i.reshape((im_height,im_width,4))
    i3 = i2[:,:,:3]
    cv2.imshow("",i3)
    cv2.waitKey(1)
    return i3/255.0
    
    