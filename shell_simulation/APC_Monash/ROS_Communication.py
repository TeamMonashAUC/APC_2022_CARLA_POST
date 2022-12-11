'''
ROS_Communication (Level 1 code)
File purpose:
    - Level 1 code means it will be used by any other level 1 code or above
    - Enables communication with incoming rostopics
    - Simplify all ros communications as simple functions


    - Does not include ros node naming with roscore
'''


'''
rosnode name: APC_Monash

rostopic used (corresponding rosmsg):
    Subscribed to
    1) /carla/ego_vehicle/vehicle_info      (provide vehicle information, especially max_steer_angle)
    2) /carla/ego_vehicle/speedometer 	    (Float32)
    3) /carla/ego_vehicle/gnss              (NavSatFix)
    4) /carla/ego_vehicle/imu               (Imu)
    5) /carla/ego_vehicle/odometry   	    (Odometry)

    Publish to
    1) /carla/ego_vehicle/vehicle_control_cmd (CarlaEgoVehicleControl)
'''


#################################################################################################################################################
# import other prgramming files
import settings # adds global variables




#################################################################################################################################################
# ros msg & libraries
import rospy
import math


'''
ROS msg are formats that ROS uses to send over topics
importing these files are needed so that we can apply their format and send to the rostopis without any issues
'''

from std_msgs.msg import String, Float64, Float32
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, Point
import tf2_ros
import tf2_geometry_msgs.tf2_geometry_msgs
from carla_msgs.msg import CarlaCollisionEvent, CarlaEgoVehicleControl, CarlaEgoVehicleInfo,CarlaEgoVehicleInfoWheel
from sensor_msgs.msg import NavSatFix, Imu






#################################################################################################################################################

'''
Function Explanation :
    Wrapper function to case any
'''
def ROS_Start():
    
    # retrieve car information from carla using rostopic
    rospy.Subscriber('/carla/ego_vehicle/vehicle_info',CarlaEgoVehicleInfo,car_data) 	 
    
    # retrieve sensor data from carla using rostopic
    rospy.Subscriber('/carla/ego_vehicle/speedometer',Float32,receive_Speedometer)   
    # rospy.Subscriber('/carla/ego_vehicle/gnss',NavSatFix,receive_Gnss)   
    # rospy.Subscriber('/carla/ego_vehicle/imu',Imu,receive_IMU)   
    rospy.Subscriber('/carla/ego_vehicle/odometry',Odometry,receive_Odometry)   

    

    ####################################################
    # publish data to carla using rostopic
    global publish_carla_data # store as global variable in this file to publish data to this node
    publish_carla_data = rospy.Publisher('/carla/ego_vehicle/vehicle_control_cmd',CarlaEgoVehicleControl, queue_size=10)
    






#################################################################################################################################################
'''
Function Explanation :
    publish rostopics directly to carla to control the car     
'''
def transmit_to_carla(car_throttle = 0, car_steer = 0, car_brake = 0, car_reverse = False, car_handBrake = False):
    global publish_carla_data
    controls = CarlaEgoVehicleControl() # apply this format to be sent using ROS
 
    # set limits for carla
    if car_throttle > 1:
        car_throttle = 1
    elif car_throttle < 0:
        car_throttle = 0    

    if car_steer > 1:
        car_steer = 1
    elif car_steer < -1:
        car_steer = -1    

    if car_brake > 1:
        car_brake = 1
    elif car_brake < 0:
        car_brake = 0    

    # set parameters to send
    controls.throttle    	 = car_throttle
    controls.steer    		 = car_steer
    controls.brake   		 = car_brake
    controls.reverse   	    = car_reverse
    controls.hand_brake     = car_handBrake

    # publish to carla
    publish_carla_data.publish(controls)
    







#################################################################################################################################################
'''
Functions to run after obtaining new ROS data from ROS topics
'''

def car_data(data):
    # global max_steer_angle
    # rospy.loginfo(data.wheels)
    settings.max_steer_angle = math.degrees(1.221730351448059)




###########################################################
def receive_Speedometer(speeed_ms): # speed given by Speedometer is in m/s
    settings.currentCarSpeed = float(speeed_ms.data)*3.6 #convert m/s to km/h


###########################################################
def receive_Gnss(gnss):
    settings.latitude = gnss.latitude
    settings.longitude = gnss.longitude
    settings.altitude = gnss.altitude
    

###########################################################
def receive_IMU(imu):
    pass


###########################################################
def receive_Odometry(data):
    settings.curr_time = data.header.stamp # obtained from "rosmsg show nav_msgs/Odometry"
    settings.car_coordinate     = [data.pose.pose.position.x, data.pose.pose.position.y, data.pose.pose.position.z]
    settings.car_direction     = [data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w]
    # rospy.loginfo(car_coordinate)
