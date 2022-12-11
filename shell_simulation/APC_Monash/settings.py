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

latitude = 0
longitude = 0
altitude = 0

curr_time =0
car_coordinate = [0,0,0]
car_direction = [0,0,0,0]
#rosdata
# ros_front_camera    = Image()
# ros_top_camera      = Image()
