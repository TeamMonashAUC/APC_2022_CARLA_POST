#!/usr/bin/env python
'''
settings (Level 1 code)
File purpose:
	- used to store all global variables between the various files
	- ensure all global variables are used instead of local variables within the files alone
'''


#################################################################################################################################################
# Libraries used
import math



#################################################################################################################################################
# ros msg & libraries
import rospy
from std_msgs.msg import String, Float64, Float32
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, Point
import tf2_ros
import tf2_geometry_msgs.tf2_geometry_msgs
from carla_msgs.msg import CarlaCollisionEvent
from carla_msgs.msg import CarlaEgoVehicleControl, CarlaEgoVehicleInfo,CarlaEgoVehicleInfoWheel
# from sensor_msgs.msg import Image

#################################################################################################################################################
# global variables
currentCarSpeed     = 0
curr_time = 0  

#car_variables
max_steer_angle = math.degrees(1.221730351448059) # about 70 degree'
wheelBase = 3  # 3m obtain from rostopic vehicle_info, by getting the difference in position of the front wheel and back wheels

curr_time = 0 # in seconds, with decimal points until nanosecond
car_coordinate_from_world = [0,0,0] # car coordinate in [x,y,z] 
car_direction_from_world = [0,0,0]  # euler angle [roll, pitch, yaw] 

turns = 0
current_quadrant = 1
prev_quadrant = 1
#################################################################################################################################################
'''
The goal types are described using numbers and are represented as follow
Moving forward
	0 - straight goal
	1 - goal before corner
	2 - corner goal
	3 - goal after corner
	4 - actual goal in the competition, its control is same as goal type 0
	5 - 3-point D
	7 - last goal, need to fully stop
Moving backward
	6 - straight goal
	8 - goal before corner
	9 - corner goal
	10 - goal after corner
	11 - reverse variant of 3-point D

	Special add-ons for 2022 APC
	12 - sharp corner turn
	13 - sharp sharp corner turn
	14 - sharp sharp sharp corner turn
	15 - straight zero throttle
'''
pose_seq 		= []   # [x, y, target direction] coordinate
pose_type		= []   # [turn type, speed limit]

# odom = Odometry ()
#rosdata
# ros_front_camera    = Image()
# ros_top_camera      = Image()
