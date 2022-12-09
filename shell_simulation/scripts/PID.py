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
from carla_msgs.msg import CarlaEgoVehicleControl


# PID library
from simple_pid import PID

# global variables
currentSpeed 	= 0

#################################################################################################################################################
def main():
	# carSetSpeed(30)
	carControl(30)



#################################################################################################################################################
# level 1 control (deals with communication with rostopics & roscore)

'''
rosnode name: APC_Monash

rostopic used (corresponding rosmsg):
	Subscribed to
	1) /carla/ego_vehicle/speedometer  	(Float32)
	2) /carla/ego_vehicle/odometry		(Odometry)

	Publish to
	1) /carla/ego_vehicle/vehicle_control_cmd (CarlaEgoVehicleControl)
'''

	#Function Explanation :
		# Initialise communication with roscore, give this rosnode name as APC_Monash 	
def listener():
	rospy.init_node('APC_Monash')
	rate = rospy.Rate(100) # publish data at 100Hz 
	rospy.loginfo("APC_Monash started")

	receive_from_carla()
	
	while not rospy.is_shutdown():
		main()
		rate.sleep()

##########################################
	#Function Explanation :
		# publish rostopics directly to carla to control the car 	
def transmit_to_carla(car_throttle = 0, car_steer = 0, car_brake = False, car_reverse = False, car_handBrake = False):
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

# def receive_Odometry():
	

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
	pid = PID(Kp = 0.4, Ki = 0, Kd = 0.2, setpoint= targetSpeed)

	# calc = pid(currentSpeed)
	return pid(currentSpeed)

##########################################
	#Function Explanation :
	# 	Steering calculations (not complete)


##########################################
	#Function Explanation :
		# allows easy input, Speed (in m/s, can be any negative or positive number), Steer (angle of travel wanted)
def carControl(targetSpeed=0, SteerAngle = 0):
	# variables to be sent to carla
	car_throttle 	= 0
	car_steer 		= 0
	car_brake 		= False
	car_reverse 	= False
	car_handBrake 	= False
	
	# Throttle control
	if targetSpeed > 0:
		car_throttle = PID_SetSpeed(abs(targetSpeed))
		car_reverse = False
	elif targetSpeed < 0:
		car_throttle = PID_SetSpeed(abs(targetSpeed))
		car_reverse = True
	else :
		car_throttle = 0
		car_reverse = False
	
	# Steering control


	rospy.loginfo("target:%d  current:%d" , targetSpeed, currentSpeed)
	transmit_to_carla(car_throttle, car_steer, car_brake, car_reverse, car_handBrake)



#################################################################################################################################################
if __name__ == '__main__':
	try:
		listener() # Call listener function when script is executed

	except rospy.ROSInterruptException: # if we stop the script, it will run rospy.ROSInterruptException
		rospy.loginfo("Exit program successful")
		pass
