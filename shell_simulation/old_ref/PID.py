#!/usr/bin/env python

# Author(s): Team Monash SEM Intelligent Department
# Documented by: Team Monash SEM Intelligent Department
# School: Monash University Malaysia
# Description: - Python script that controls the car in Simulator from start to finish
#                by subscribing to the car's odometry data, calculating the required throttle,
#                gear & steering values based on pre-defined goal sequences and goal types,
#                and publish these values to the simulator to move the car at a defined period.
#              - This _simple variant simplifies the throttle to basic increments and decrements for low speeds (<= 5m/s)


#################################################################################################################################################
# install instruction

# libraries install 
'''
simple pid  -  "pip install simple-pid"
'''

#################################################################################################################################################
# ros msg & libraries
import rospy
from std_msgs.msg import String, Float64, Float32
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, Point
import math
import tf2_ros
import tf2_geometry_msgs.tf2_geometry_msgs
from carla_msgs.msg import CarlaCollisionEvent
from carla_msgs.msg import CarlaEgoVehicleControl, CarlaEgoVehicleInfo,CarlaEgoVehicleInfoWheel


# PID library
''' How to use https://pypi.org/project/simple-pid/'''
from simple_pid import PID

# global variables
currentSpeed 	= 0
curr_time = 0

#car_variables
max_steer_angle = math.degrees(1.221730351448059) # about 70 degree


#################################################################################################################################################
def main():
	carControl(10, 0)
	# CarlaEgoVehicleInfo.wheels
	# CarlaEgoVehicleInfoWheel.max_steer_angle
	# link = rospy.Subscriber('/carla/ego_vehicle/odometry',Odometry,receive_Odometry)
	# tf2_buffer = tf2_ros.Buffer()
	# goal_map = PoseStamped()

	# goal_map.pose.position.x = 0
	# goal_map.pose.position.y = 0
	# trans = tf2_buffer.lookup_transform("ego_vehicle","map",rospy.Time())
	# output = tf2_buffer.transform(goal_map,"ego_vehicle")
	# rospy.loginfo(output)
# def base(data):
	
	# rospy.loginfo(data.wheels)
	# rospy.loginfo(data.max_steer_angle)

#################################################################################################################################################
# level 1 control (deals with communication with rostopics & roscore)

'''
rosnode name: APC_Monash

rostopic used (corresponding rosmsg):
	Subscribed to
	1) /carla/ego_vehicle/speedometer  	(Float32)
	2) /carla/ego_vehicle/odometry		(Odometry)
	3) /carla/ego_vehicle/vehicle_info  (provide vehicle information, especially max_steer_angle)

	Publish to
	1) /carla/ego_vehicle/vehicle_control_cmd (CarlaEgoVehicleControl)
'''

	#Function Explanation :
		# Initialise communication with roscore, give this rosnode name as APC_Monash 	
def listener():
	rospy.init_node('APC_Monash')
	rate = rospy.Rate(100) # publish data at 100Hz 
	rospy.loginfo("APC_Monash started")

	obtain_car_info()

	receive_from_carla()
	
	while not rospy.is_shutdown():
		main()
		rate.sleep()

##########################################
	#Function Explanation :
		# Obtain steering angle from simulator (one time thing) (not complete)
def obtain_car_info():
	rospy.Subscriber('/carla/ego_vehicle/vehicle_info',CarlaEgoVehicleInfo,car_data)

def car_data(data):
	global max_steer_angle
	# rospy.loginfo(data.wheels)
	max_steer_angle = math.degrees(1.221730351448059)

##########################################
	#Function Explanation :
		# publish rostopics directly to carla to control the car 	
def transmit_to_carla(car_throttle = 0, car_steer = 0, car_brake = 0, car_reverse = False, car_handBrake = False):
	pub = rospy.Publisher('/carla/ego_vehicle/vehicle_control_cmd',CarlaEgoVehicleControl, queue_size=10)
	controls = CarlaEgoVehicleControl()
 
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
	controls.throttle 		= car_throttle 
	controls.steer 			= car_steer
	controls.brake			= car_brake
	controls.reverse		= car_reverse 
	controls.hand_brake 	= car_handBrake

	# publish to carla
	pub.publish(controls)
	
##########################################
	#Function Explanation :
		# receive rostopics from carla environment and convert it as inputs
def receive_from_carla():
	rospy.Subscriber('/carla/ego_vehicle/speedometer',Float32,receive_Speedometer)
	# rospy.Subscriber('/carla/ego_vehicle/odometry',Odometry,receive_Odometry)

def receive_Speedometer(speeed_ms): # speed given by Speedometer is in m/s
	global currentSpeed
	currentSpeed = float(speeed_ms.data)*3.6 #convert m/s to km/h

def receive_Odometry(data):
	global curr_time, car_coordinate, car_direction
	curr_time = data.header.stamp # obtained from "rosmsg show nav_msgs/Odometry"
	car_coordinate 	= [data.pose.pose.position.x, data.pose.pose.position.y, data.pose.pose.position.z]
	car_direction 	= [data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w]
	rospy.loginfo(car_direction)

#################################################################################################################################################
# level 2 control (calculate speed and steering angle)
'''
Purpose
- easier use for higher level programming
- standard unit of km/h
'''

##########################################
# 	output:
# 		throttle

	#Function Explanation :
		# uses the target speed to calculate throttle using PID and return the throttle value as an output
def PID_SetSpeed(targetSpeed):
	global currentSpeed

	# PID parameters setting
	pid = PID(Kp = 0.4, Ki = 0.3, Kd = 0.2, setpoint= targetSpeed)

	# calc = pid(currentSpeed)
	return pid(currentSpeed)

##########################################
	#Function Explanation :
	# 	arduino map function, converts the range of the input and output 	
	'''
	arduino map function https://cdn.arduino.cc/reference/en/language/functions/math/map/
	for arduino map function in python https://www.theamplituhedron.com/articles/How-to-replicate-the-Arduino-map-function-in-Python-for-Raspberry-Pi/
	'''
def arduino_map_function(x, in_min, in_max, out_min, out_max):
    return float((x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min)

##########################################
	#Function Explanation :
		# allows easy input, Speed (in m/s, can be any negative or positive number), Steer (angle of travel wanted)
def carControl(targetSpeed = 0, SteerAngle = 0):
	# variables to be sent to carla
	car_throttle 	= 0
	car_steer 		= 0
	car_brake 		= False
	car_reverse 	= False
	car_handBrake 	= False
	
	# Throttle control
	calc = PID_SetSpeed(targetSpeed) # calculate PID values, decide later if we should break or accelerate
	if targetSpeed > 0:
		car_reverse = False

		if calc >= 0: 	# accelerate
			car_throttle = abs(calc)
		else:			# brake
			car_brake = abs(calc)

	elif targetSpeed < 0: 
		car_reverse = True
		
		if calc <= 0:	# reverse
			car_throttle = abs(calc)
		else:			# brake
			car_brake = abs(calc)
	else :
		car_throttle = 0
		car_brake = 1
		car_reverse = False
	
	# Steering control
	car_steer = arduino_map_function(SteerAngle,-max_steer_angle ,max_steer_angle ,-1,1) # convert range in angle to steering angle (-70,70) -> (-1,1)

	# rospy.loginfo("target:%d  current:%d car_brake:%f" , targetSpeed, currentSpeed,car_brake)
	transmit_to_carla(car_throttle, car_steer, car_brake, car_reverse, car_handBrake)



#################################################################################################################################################
if __name__ == '__main__':
	try:
		listener() # Call listener function when script is executed

	except rospy.ROSInterruptException: # if we stop the script, it will run rospy.ROSInterruptException
		rospy.loginfo("Exit program successful")
		pass
