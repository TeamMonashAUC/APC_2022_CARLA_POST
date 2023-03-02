#!/usr/bin/env python
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
import shell_simulation_2.settings as settings # adds global variables


from shell_simulation.msg import ActualCoord


#################################################################################################################################################
# ros msg & libraries
import rospy
from shell_simulation.msg import Score

import math


'''
ROS msg are formats that ROS uses to send over topics
importing these files are needed so that we can apply their format and send to the rostopis without any issues
'''

from std_msgs.msg import String, Float64, Float32,Bool
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, Point
import tf2_ros
import tf2_geometry_msgs.tf2_geometry_msgs
from carla_msgs.msg import CarlaCollisionEvent, CarlaEgoVehicleControl, CarlaEgoVehicleInfo,CarlaEgoVehicleInfoWheel
from sensor_msgs.msg import NavSatFix, Imu


# from geometry_msgs.msg import PoseStamped, Point
# import tf2_ros
# import tf2_geometry_msgs.tf2_geometry_msgs




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
    rospy.Subscriber("/Score", Score,receive_Score)   
    

    ####################################################
    ''' 
    # used this to talk to carla directly

    # publish data to carla using rostopic
    global publish_carla_data # store as global variable in this file to publish data to this node
    
    publish_carla_data = rospy.Publisher('/carla/ego_vehicle/vehicle_control_cmd',CarlaEgoVehicleControl, queue_size=10)
    '''
    ####################################################
    '''
    for APC 2023, we are required to write our car control data to the following topics
    '''
    global pub_gear
    global pub_brake
    global pub_steering
    global pub_throttle
    global pub_handbrake

    pub_gear = rospy.Publisher("/gear_command", String, queue_size = 1)
    pub_throttle = rospy.Publisher("/throttle_command", Float64, queue_size = 1)
    pub_steering = rospy.Publisher("/steering_command", Float64, queue_size = 1)
    pub_brake = rospy.Publisher("/brake_command", Float64, queue_size = 1)
    pub_handbrake = rospy.Publisher("/handbrake_command", Bool, queue_size = 1)

    
    global throttle_data
    global steering_data 
    global brake_data
    global gear_data
    global handbrake_data

    throttle_data = steering_data = brake_data =  Float64()
    gear_data = String()
    handbrake_data = Bool()
    ####################################################

    # publish coordinate data
    global pub_coord_2D
    pub_coord_2D = rospy.Publisher("/actual_coord_2D", ActualCoord, queue_size =10000, latch=False)

    global pub_coord_3D
    pub_coord_3D = rospy.Publisher("/actual_coord_3D", ActualCoord, queue_size =10000, latch=False)



################################################################################################################################################# 
'''
Function Explanation :
    publish rostopics directly to carla to control the car     
'''
def transmit_to_carla(car_throttle = 0, car_steer = 0, car_brake = 0, car_reverse = False, car_handBrake = False):
 
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

    ''' 
    # set parameters to send to carla directly
    global publish_carla_data
    controls = CarlaEgoVehicleControl() # apply this format to be sent using ROS
    
    controls.throttle    	 = car_throttle
    controls.steer    		 = car_steer
    controls.brake   		 = car_brake
    controls.reverse   	    = car_reverse
    controls.hand_brake     = car_handBrake

    # publish to carla
    publish_carla_data.publish(controls)

    '''

    ########################################################
    # publish data as per the rules for APC 2023

    global throttle_data
    global steering_data 
    global brake_data
    global gear_data
    global handbrake_data

    throttle_data = steering_data = brake_data =  Float64()
    gear_data = String()
    handbrake_data = Bool()

    throttle_data = car_throttle
    steering_data = car_steer
    brake_data = car_brake
    handbrake_data = car_handBrake
    
    
    pub_throttle.publish(throttle_data)
    pub_steering.publish(steering_data)
    pub_brake.publish(brake_data)
    pub_handbrake.publish(handbrake_data)

    if (car_reverse == True):
        pub_gear.publish("reverse")
    else :
        pub_gear.publish("forward")

    # publish current car coordinate after sending driving info
    coord_publish()
  








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
    # settings.latitude = gnss.latitude
    # settings.longitude = gnss.longitudepenaltyCounter
    # settings.altitude = gnss.altitude
    pass

###########################################################
def receive_IMU(imu):
    pass


###########################################################
def receive_Odometry(data):
    settings.curr_time = data.header.stamp # obtained from "rosmsg show nav_msgs/Odometry"
    settings.car_coordinate_from_world     = [data.pose.pose.position.x, data.pose.pose.position.y, data.pose.pose.position.z]

    settings.curr_time = data.header.stamp.secs + (data.header.stamp.nsecs)*1e-9

    # settings.car_direction_from_world      = euler_from_quaternion(data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w)
    euler_angle = euler_from_quaternion(data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w)
    # settings.car_direction_from_world      = [euler_angle[0],euler_angle[1],absoluteYaw(euler_angle[2])]
    settings.car_direction_from_world      = [euler_angle[0],euler_angle[1],euler_angle[2]]
    # rospy.loginfo(car_coordinate)
    
    # rospy.loginfo(data.header)

    # # Goal point transformer
    # goal_map = PoseStamped ()
    # tf2_buffer = tf2_ros.Buffer()
    # listener = tf2_ros.TransformListener(tf2_buffer)

    # # goal_map = PoseStamped()
    # goal_map.header = data.header
    # # rospy.loginfo(data)
    # goal_map.pose.position.x = -77.8
    # goal_map.pose.position.y = 50
    
    # #Transform goal points in map frame 'map' to car frame 'ego_vehicle
    # # rospy.loginfo(output)
    # while not rospy.is_shutdown():
    # try:
    #     trans = tf2_buffer.lookup_transform("ego_vehicle","map",rospy.Time())
    #     # rospy.loginfo(trans)
    
    # except(tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
    #     rospy.sleep(0.001)
        
    # while not rospy.is_shutdown():
    # output=0
    # try:
    #     trans = tf2_buffer.lookup_transform("ego_vehicle","map",rospy.Time())
    #     output = tf2_buffer.transform(goal_map,"ego_vehicle")
    #     rospy.loginfo(output)
    
    # except:
    #     rospy.sleep(0.001)
        
    # rospy.loginfo(output)
    
    # rospy.loginfo(output.pose.position.x)
    # tf2_buffer.lookup_transform("ego_vehicle","map",rospy.Time())

    
###########################################################
'''
Functions to convert -180 to 180 angle to infinite angle to make calculations easier
'''
# turns = 0    
# current_quadrant = 1
# prev_quadrant = 1
def absoluteYaw(angle):
    
    settings.prev_quadrant = settings.current_quadrant
    if (angle >=0 and angle <90):
        settings.current_quadrant = 1 
    elif (angle >=90 and angle <=180):
        settings.current_quadrant = 2
    elif (angle <-90 and angle >=-180):
        settings.current_quadrant = 3
    elif (angle <0 and angle >=-90):
        settings.current_quadrant = 4
   
    
    if(settings.prev_quadrant==3 and settings.current_quadrant==2):
        settings.turns = settings.turns - 1

    elif(settings.prev_quadrant==2 and settings.current_quadrant==3):
        settings.turns = settings.turns + 1

    absolute_angle = settings.turns*360 + angle  
    # rospy.loginfo(settings.turns)
    # rospy.loginfo(absolute_angle)

    return absolute_angle
###########################################################
'''
Functions to convert quaternion angle to euler angle
'''
def euler_from_quaternion(x, y, z, w):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
     
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
     
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)

        degree_roll_x = roll_x*180/math.pi        
        degree_pitch_y= pitch_y*180/math.pi
        degree_yaw_z = yaw_z*180/math.pi
        
        return degree_roll_x, degree_pitch_y, degree_yaw_z # in degree

###########################################################



def receive_Score(data):
    # data.closest_approach = []		# goal distance
    settings.coord_distance = data.closest_approach
    # rospy.loginfo(settings.coord_distance)


def coord_publish():
    global pub_coord_2D 
    
    msg_2d = ActualCoord()
    msg_2d.header.stamp.secs=int(settings.curr_time)
    msg_2d.header.stamp.nsecs= int((settings.curr_time - int(settings.curr_time))*1e9)
    msg_2d.length = len(settings.coord_2d)-1

    # rospy.loginfo(len(settings.coord_2d))
    if(len(settings.coord_2d)>=19):
        # rospy.loginfo(len(settings.coord_2d))
        rospy.loginfo(settings.coord_2d)
        msg_2d.P0 = settings.coord_2d[1][0]
        msg_2d.P1 = settings.coord_2d[2][0]
        msg_2d.P2 = settings.coord_2d[3][0]
        msg_2d.P3 = settings.coord_2d[4][0]
        msg_2d.P4 = settings.coord_2d[5][0]
        msg_2d.P5 = settings.coord_2d[6][0]
        msg_2d.P6 = settings.coord_2d[7][0]
        msg_2d.P7 = settings.coord_2d[8][0]
        msg_2d.P8 = settings.coord_2d[9][0]
        msg_2d.P9 = settings.coord_2d[10][0]
        msg_2d.P10 = settings.coord_2d[11][0]
        msg_2d.P11 = settings.coord_2d[12][0]
        msg_2d.P12 = settings.coord_2d[13][0]
        msg_2d.P13 = settings.coord_2d[14][0]
        msg_2d.P14 = settings.coord_2d[15][0]
        msg_2d.P15 = settings.coord_2d[16][0]
        msg_2d.P16 = settings.coord_2d[17][0]
    elif(len(settings.coord_2d)>=16):
        msg_2d.P0 = settings.coord_2d[1][0]
        msg_2d.P1 = settings.coord_2d[2][0]
        msg_2d.P2 = settings.coord_2d[3][0]
        msg_2d.P3 = settings.coord_2d[4][0]
        msg_2d.P4 = settings.coord_2d[5][0]
        msg_2d.P5 = settings.coord_2d[6][0]
        msg_2d.P6 = settings.coord_2d[7][0]
        msg_2d.P7 = settings.coord_2d[8][0]
        msg_2d.P8 = settings.coord_2d[9][0]
        msg_2d.P9 = settings.coord_2d[10][0]
        msg_2d.P10 = settings.coord_2d[11][0]
        msg_2d.P11 = settings.coord_2d[12][0]
        msg_2d.P12 = settings.coord_2d[13][0]
        msg_2d.P13 = settings.coord_2d[14][0]
        msg_2d.P14 = settings.coord_2d[15][0]
    elif(len(settings.coord_2d)>=15):
            msg_2d.P0 = settings.coord_2d[1][0]
            msg_2d.P1 = settings.coord_2d[2][0]
            msg_2d.P2 = settings.coord_2d[3][0]
            msg_2d.P3 = settings.coord_2d[4][0]
            msg_2d.P4 = settings.coord_2d[5][0]
            msg_2d.P5 = settings.coord_2d[6][0]
            msg_2d.P6 = settings.coord_2d[7][0]
            msg_2d.P7 = settings.coord_2d[8][0]
            msg_2d.P8 = settings.coord_2d[9][0]
            msg_2d.P9 = settings.coord_2d[10][0]
            msg_2d.P10 = settings.coord_2d[11][0]
            msg_2d.P11 = settings.coord_2d[12][0]
            msg_2d.P12 = settings.coord_2d[13][0]
            msg_2d.P13 = settings.coord_2d[14][0]
    elif(len(settings.coord_2d)>=14):
            msg_2d.P0 = settings.coord_2d[1][0]
            msg_2d.P1 = settings.coord_2d[2][0]
            msg_2d.P2 = settings.coord_2d[3][0]
            msg_2d.P3 = settings.coord_2d[4][0]
            msg_2d.P4 = settings.coord_2d[5][0]
            msg_2d.P5 = settings.coord_2d[6][0]
            msg_2d.P6 = settings.coord_2d[7][0]
            msg_2d.P7 = settings.coord_2d[8][0]
            msg_2d.P8 = settings.coord_2d[9][0]
            msg_2d.P9 = settings.coord_2d[10][0]
            msg_2d.P10 = settings.coord_2d[11][0]
            msg_2d.P11 = settings.coord_2d[12][0]
            msg_2d.P12 = settings.coord_2d[13][0]
    elif(len(settings.coord_2d)>=13):
            msg_2d.P0 = settings.coord_2d[1][0]
            msg_2d.P1 = settings.coord_2d[2][0]
            msg_2d.P2 = settings.coord_2d[3][0]
            msg_2d.P3 = settings.coord_2d[4][0]
            msg_2d.P4 = settings.coord_2d[5][0]
            msg_2d.P5 = settings.coord_2d[6][0]
            msg_2d.P6 = settings.coord_2d[7][0]
            msg_2d.P7 = settings.coord_2d[8][0]
            msg_2d.P8 = settings.coord_2d[9][0]
            msg_2d.P9 = settings.coord_2d[10][0]
            msg_2d.P10 = settings.coord_2d[11][0]
            msg_2d.P11 = settings.coord_2d[12][0]
    elif(len(settings.coord_2d)>=12):
            msg_2d.P0 = settings.coord_2d[1][0]
            msg_2d.P1 = settings.coord_2d[2][0]
            msg_2d.P2 = settings.coord_2d[3][0]
            msg_2d.P3 = settings.coord_2d[4][0]
            msg_2d.P4 = settings.coord_2d[5][0]
            msg_2d.P5 = settings.coord_2d[6][0]
            msg_2d.P6 = settings.coord_2d[7][0]
            msg_2d.P7 = settings.coord_2d[8][0]
            msg_2d.P8 = settings.coord_2d[9][0]
            msg_2d.P9 = settings.coord_2d[10][0]
            msg_2d.P10 = settings.coord_2d[11][0]

    pub_coord_2D.publish(msg_2d)







    global pub_coord_3D 
    
    msg_3d = ActualCoord()
    msg_3d.header.stamp.secs=int(settings.curr_time)
    msg_3d.header.stamp.nsecs= int((settings.curr_time - int(settings.curr_time))*1e9)
    msg_3d.length = len(settings.coord_3d)-1

    # rospy.loginfo(len(settings.coord_3d))
    if(len(settings.coord_3d)>=19):
        # rospy.loginfo(len(settings.coord_3d))
        rospy.loginfo(settings.coord_3d)
        msg_3d.P0 = settings.coord_3d[1][0]
        msg_3d.P1 = settings.coord_3d[2][0]
        msg_3d.P2 = settings.coord_3d[3][0]
        msg_3d.P3 = settings.coord_3d[4][0]
        msg_3d.P4 = settings.coord_3d[5][0]
        msg_3d.P5 = settings.coord_3d[6][0]
        msg_3d.P6 = settings.coord_3d[7][0]
        msg_3d.P7 = settings.coord_3d[8][0]
        msg_3d.P8 = settings.coord_3d[9][0]
        msg_3d.P9 = settings.coord_3d[10][0]
        msg_3d.P10 = settings.coord_3d[11][0]
        msg_3d.P11 = settings.coord_3d[12][0]
        msg_3d.P12 = settings.coord_3d[13][0]
        msg_3d.P13 = settings.coord_3d[14][0]
        msg_3d.P14 = settings.coord_3d[15][0]
        msg_3d.P15 = settings.coord_3d[16][0]
        msg_3d.P16 = settings.coord_3d[17][0]
    elif(len(settings.coord_3d)>=16):
        msg_3d.P0 = settings.coord_3d[1][0]
        msg_3d.P1 = settings.coord_3d[2][0]
        msg_3d.P2 = settings.coord_3d[3][0]
        msg_3d.P3 = settings.coord_3d[4][0]
        msg_3d.P4 = settings.coord_3d[5][0]
        msg_3d.P5 = settings.coord_3d[6][0]
        msg_3d.P6 = settings.coord_3d[7][0]
        msg_3d.P7 = settings.coord_3d[8][0]
        msg_3d.P8 = settings.coord_3d[9][0]
        msg_3d.P9 = settings.coord_3d[10][0]
        msg_3d.P10 = settings.coord_3d[11][0]
        msg_3d.P11 = settings.coord_3d[12][0]
        msg_3d.P12 = settings.coord_3d[13][0]
        msg_3d.P13 = settings.coord_3d[14][0]
        msg_3d.P14 = settings.coord_3d[15][0]
    elif(len(settings.coord_3d)>=15):
            msg_3d.P0 = settings.coord_3d[1][0]
            msg_3d.P1 = settings.coord_3d[2][0]
            msg_3d.P2 = settings.coord_3d[3][0]
            msg_3d.P3 = settings.coord_3d[4][0]
            msg_3d.P4 = settings.coord_3d[5][0]
            msg_3d.P5 = settings.coord_3d[6][0]
            msg_3d.P6 = settings.coord_3d[7][0]
            msg_3d.P7 = settings.coord_3d[8][0]
            msg_3d.P8 = settings.coord_3d[9][0]
            msg_3d.P9 = settings.coord_3d[10][0]
            msg_3d.P10 = settings.coord_3d[11][0]
            msg_3d.P11 = settings.coord_3d[12][0]
            msg_3d.P12 = settings.coord_3d[13][0]
            msg_3d.P13 = settings.coord_3d[14][0]
    elif(len(settings.coord_3d)>=14):
            msg_3d.P0 = settings.coord_3d[1][0]
            msg_3d.P1 = settings.coord_3d[2][0]
            msg_3d.P2 = settings.coord_3d[3][0]
            msg_3d.P3 = settings.coord_3d[4][0]
            msg_3d.P4 = settings.coord_3d[5][0]
            msg_3d.P5 = settings.coord_3d[6][0]
            msg_3d.P6 = settings.coord_3d[7][0]
            msg_3d.P7 = settings.coord_3d[8][0]
            msg_3d.P8 = settings.coord_3d[9][0]
            msg_3d.P9 = settings.coord_3d[10][0]
            msg_3d.P10 = settings.coord_3d[11][0]
            msg_3d.P11 = settings.coord_3d[12][0]
            msg_3d.P12 = settings.coord_3d[13][0]
    elif(len(settings.coord_3d)>=13):
            msg_3d.P0 = settings.coord_3d[1][0]
            msg_3d.P1 = settings.coord_3d[2][0]
            msg_3d.P2 = settings.coord_3d[3][0]
            msg_3d.P3 = settings.coord_3d[4][0]
            msg_3d.P4 = settings.coord_3d[5][0]
            msg_3d.P5 = settings.coord_3d[6][0]
            msg_3d.P6 = settings.coord_3d[7][0]
            msg_3d.P7 = settings.coord_3d[8][0]
            msg_3d.P8 = settings.coord_3d[9][0]
            msg_3d.P9 = settings.coord_3d[10][0]
            msg_3d.P10 = settings.coord_3d[11][0]
            msg_3d.P11 = settings.coord_3d[12][0]
    elif(len(settings.coord_3d)>=12):
            msg_3d.P0 = settings.coord_3d[1][0]
            msg_3d.P1 = settings.coord_3d[2][0]
            msg_3d.P2 = settings.coord_3d[3][0]
            msg_3d.P3 = settings.coord_3d[4][0]
            msg_3d.P4 = settings.coord_3d[5][0]
            msg_3d.P5 = settings.coord_3d[6][0]
            msg_3d.P6 = settings.coord_3d[7][0]
            msg_3d.P7 = settings.coord_3d[8][0]
            msg_3d.P8 = settings.coord_3d[9][0]
            msg_3d.P9 = settings.coord_3d[10][0]
            msg_3d.P10 = settings.coord_3d[11][0]

    pub_coord_3D.publish(msg_3d)