#!/usr/bin/env python
import rospy
from std_msgs.msg import String, Float64
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, Point
import math
import tf2_ros
import tf2_geometry_msgs.tf2_geometry_msgs
from carla_msgs.msg import CarlaCollisionEvent

# car_control_master.py
# Author(s): Team Monash SEM Intelligent Department
# Documented by: Team Monash SEM Intelligent Department
# School: Monash University Malaysia
# Description: - Python script that controls the car in Simulator from start to finish
#                by subscribing to the car's odometry data, calculating the required throttle,
#                gear & steering values based on pre-defined goal sequences and goal types,
#                and publish these values to the simulator to move the car at a defined period.
#              - This _simple variant simplifies the throttle to basic increments and decrements for low speeds (<= 5m/s)

class Control: # Control class for modular code
	def __init__(self): # Class constructor
		# Constants
		self.fct = [0.35, 0.005, 0.02] # Throttle factors
		self.rad90 = math.radians(90) # 90 degrees in radians
		self.b_wheel_base = 2.9 # Wheelbase length [m]

		# Main parameters
		self.poll_period = 0.2 # Period of calling callback() function
		self.config = 1 # Goal sequence configuration

		# Initialize method attributes (variables global to class)
		self.car_x = self.car_y = self.car_z = self.yaw = self.v_x = self.v_y = self.v_z = self.t_poll = self.t_tot = self.t0 = self.throttle = self.steering = self.brake = self.prev_gas = self.stop_cal_t = 0
		self.stop = self.end = self.start = self.move = False
		self.prev_gear = "forward"
		self.prev_goal_type = self.goal_type = -1
		self.crash = False
		self.recover = False
		self.cnt = 0
		self.goal15check = False
		self.prevthrottle = 0.5 
		self.prevbrake = 0 

		# Initialize publishers and messages
		self.pub_gear = rospy.Publisher("/gear_command", String, queue_size = 1)
		self.pub_throttle = rospy.Publisher("/throttle_command", Float64, queue_size = 1)
		self.pub_steering = rospy.Publisher("/steering_command", Float64, queue_size = 1)
		self.pub_brake = rospy.Publisher("/brake_command", Float64, queue_size = 1)
		self.throttle_data = self.steering_data = self.brake_data =  Float64()
		self.gear_data = String()

		# Goal point transformer
		self.goal_map = PoseStamped ()
		self.tf2_buffer = tf2_ros.Buffer()
		self.listener = tf2_ros.TransformListener(self.tf2_buffer)

	# Class method that performs the calculation for controlling the car, total time elapsed from controller start is passed
	# Method Name: callback

	# Input :
	# 1) time: The total time elapsed from start

	# Output :
	# 1) None

	# Function Explanation :
	# Based on the current data, make decision and publish data to topics of the actuators
	# Decide when to stop finding for next goal
	# Transform goals from map frame to car frame
	# Calculate velocity and radial distance from car to goal
	# Call necessary functions/methods to calculate steering, throttle and brake value required based on current goal type
	def callback(self, time):
		gear = "forward" # Initialize gear to forward
		
		
		'''Decide when to stop finding for next goal''' 
		if self.stop and not self.end: # Exit control when reached final goal
			#rospy.loginfo("Controller Time: %.4f" %(time)) # Display total controller time
			self.end = True
			return
		elif self.end: # End condition
			return

		'''Convert goals in map coordinate to coordinate in respect to car'''
		# Transform goal points in map frame 'map' to car frame 'ego_vehicle
		while not rospy.is_shutdown():
			try:
				#trans = self.tf2_buffer.lookup_transform("ego_vehicle","map",rospy.Time())
				output = self.tf2_buffer.transform(self.goal_map,"ego_vehicle")
				break
			
			except:
				rospy.sleep(0.001)
				continue



		# Calculate velocity as a magnitude of cartesian vectors
		car_speed = math.sqrt(math.pow(self.v_x, 2) + math.pow(self.v_y, 2) + math.pow(self.v_z, 2))

		# Calculate radial distance from car to goal
		diff_radius = math.sqrt(math.pow(output.pose.position.x, 2) + math.pow(output.pose.position.y, 2))



		# Stop calculating steering when it reaches steady state
		if abs(self.steering) <= 0.2 and self.goal_type == self.prev_goal_type:
			self.stop_cal_t += 1
		else:
			self.stop_cal_t = 0
			self.prev_goal_type = self.goal_type

		rospy.loginfo("Test 1")
		rospy.loginfo("Current Round Goal type: %d", self.goal_type)
		# Steering control
		# if car_speed > 1 and (self.stop_cal_t <= 10 or (self.stop_cal_t - 10) % 10 == 0): # Settling time = 10 x poll_period

		# 	a = math.atan2(output.pose.position.y,output.pose.position.x)  # alpha
		# 	omega = 1 * a # Scalar constant to define angular velocity omega
		# 	if self.goal_type == 12:  #sharp corner turn
		# 		omega = 2.5 * a
		# 	elif self.goal_type == 13:  #sharp sharp corner turn
		# 		omega = 5 * a
		# 	elif self.goal_type == 14:  #sharp sharp sharp corner turn
		# 		omega = 8 * a
		# 	if omega != 0:
		# 		Apply Ackermann's steering
		# 		r = car_speed / -omega
		# 		self.steering = math.atan(self.b_wheel_base / r)

		# if self.goal_type in [6,8,9,10,11]: # Reverse goal types
		# 	gear = "reverse"

		# if self.goal_type == 7 and diff_radius < 3: # End condition
		# 	self.stop = True
		# 	self.steering = 0

		# Throttle control
		# if self.goal15check == True:
		# 	self.throttle = self.prevthrottle
		# 	self.goal15check = False
		# if car_speed < 0.5 :  # Low speed condition
		# 	self.throttle = 0.5
		# elif not self.stop:  # not at final goal
		# 	self.move = True # Flag when car starts moving
		# 	if self.goal_type not in [5, 11]: # Not at stop & go goal
		# 		if self.goal_type == 15:
		# 			self.goal15check = True
		# 			if self.throttle != 0 and self.prevthrottle != 0:
		# 				self.prevthrottle = self.throttle
		# 			self.throttle = 0 # make throttle 0
		# 		elif car_speed < 2:
		# 			self.throttle = self.fct[0] # Base throttle
		# 		elif self.throttle < 0.49: # Limit max throttle
		# 			self.throttle += self.fct[1] # increment throttle
		# 	else:

		# 		if diff_radius < 20 and self.throttle > 0.1:
		# 			self.throttle -= self.fct[2] # decrement throttle when close to stop & go, min 0.1
		# else: # End condition
		# 	self.throttle = 0

		# if abs(self.steering) > 0.6: # Limit max steering
		# 	self.steering = 0.6*abs(self.steering) / self.steering


		if self.goal_type in [1,2,3,8,9,10]:
			rospy.loginfo("Approaching goal: [%.2f, %.2f], Goal type: %d", self.goal_x, self.goal_y, self.goal_type)
			gear = self.corner(output, car_speed)
			rospy.loginfo("Finished running corner method")

		if self.goal_type  == 7: #final goal
			rospy.loginfo("Approaching goal: [%.2f, %.2f], Goal type: %d", self.goal_x, self.goal_y, self.goal_type)
			gear = self.finalgoal(output, car_speed, diff_radius)
			rospy.loginfo("Finished running Final Goal")

		if self.goal_type in [0, 6]: #straight
			rospy.loginfo("Test 2")
			rospy.loginfo("Approaching goal: [%.2f, %.2f], Goal type: %d", self.goal_x, self.goal_y, self.goal_type)
			gear = self.straight(output, car_speed)
			rospy.loginfo("Finished running Straight")



		# #rospy.loginfo("Collsion is %s", self.crash)
		# if self.crash == True or self.recover == True: ########## PROTOCOLS FOR CRASHING ###########
		# 	self.gear = "reverse"
		# 	self.throttle = 0.5
		# 	self.steering = 0
		# 	self.cnt += 1
		# 	if self.cnt > 10:
		# 		# if self.cnt == 6:
		# 		# 	rospy.sleep(2)
				
		# 		self.crash = True
		# 		self.recover = True
				
		# 		if self.cnt <= 20:
		# 			self.gear = "forward"
		# 			self.throttle = 1
		# 			self.steering = 0
		# 		if self.cnt > 20:
		# 			self.recover = False
		# 			self.crash = False
		# 			self.cnt = 0
		# 		# 	self.throttle = 0.7
		# 		# 	self.steering = -0.5
		# 		# elif self.cnt > 28 and self.cnt <=34:
		# 		# 	self.gear = "forward"
		# 		# 	self.throttle = 0.5
		# 		# 	self.steering = 0.4
		# 		# elif self.cnt > 34 and self.cnt <=37:
		# 		# 	self.gear = "forward"
		# 		# 	self.throttle = 0.5
		# 		# 	self.steering = 0
		# 		# elif self.cnt > 37 and self.cnt <=41:
		# 		# 	self.gear = "forward"
		# 		# 	self.throttle = 0.5
		# 		# 	self.steering = 0.4
		# 		# elif self.cnt > 41 and self.cnt <=45:
		# 		# 	self.gear = "forward"
		# 		# 	self.throttle = 0.5
		# 		# 	self.steering = -0.5
		# 		# elif self.cnt > 45 and self.cnt <=47:
		# 		# 	self.gear = "forward"
		# 		# 	self.throttle = 0.5
		# 		# 	self.steering = 0
		# 		# 	self.cnt = 0
		# 		# 	self.recover = False	
		# 	self.pub_gear.publish(self.gear)
		# 	self.pub_throttle.publish(self.throttle)
		# 	self.pub_steering.publish(self.steering)
		### Publish controls ###
		else:
			if not self.move: # Always publish gear at the start to prevent synching issues
				self.gear_data.data = gear
				self.pub_gear.publish(self.gear_data)
				# self.throttle_data.data = self.throttle
				# self.pub_throttle.publish(self.throttle_data)

			else: # Publish gear and throttle only during changes
				if gear != self.prev_gear: # Switching between forward and reverse
					# self.throttle = 1 # 'Brake' the car by counter-throttling
					# self.steering = 0 # Reset steering
					self.gear_data.data = gear
					self.pub_gear.publish(self.gear_data)
					self.prev_gear = gear # update previous gear
					# self.throttle_data.data = self.throttle
					# self.pub_throttle.publish(self.throttle_data)
					

				# if self.prev_gas != self.throttle:
				# 	self.throttle_data.data = self.throttle
				# 	self.pub_throttle.publish(self.throttle_data)
				# 	self.prev_gas = self.throttle # update previous throttle

		if self.stop_cal_t <= 10 or (self.stop_cal_t - 10) % 10 == 0: # Stop publishing steering when reached steady state
			self.steering_data.data = self.steering 
			self.pub_steering.publish(self.steering_data)

		#rospy.loginfo("Publishing: [Throttle:  %f, Brake: %f, Gear: %s, Speed_cur: %f, steer: %f, goal_type: %d, diff_radius: %f, pos_x: %f, pos_y: %f, pos_z: %f, rz: %f]" %(self.throttle, 0,gear,car_speed,self.steering,self.goal_type,diff_radius,self.car_x,self.car_y,self.car_z,self.yaw))

	# Method Name: publishThrotBrake

	# Input :
	# 1) throttle: A throttle value from min 0 to max 1 (from no throttle to maximum throttle)
	# 2) brake: A brake value from min 0 to max 1 (from no throttle to maximum throttle)

	# Output :
	# None

	# Function Explanation :
	# To Publish Throttle and Brake to the car
	def publishThrotBrake(self, throttle = 0, brake = 0):
		if not self.move: # Always publish brake & throttle at the start to prevent synching issues
				self.brake_data.data = self.brake = brake
				self.pub_brake.publish(self.brake_data)
				self.throttle_data.data = self.throttle
				self.pub_throttle.publish(self.throttle_data)

		else: # Publish brake and throttle only during changes
			if self.prevbrake != self.brake: # Switching between forward and reverse
				# self.throttle = 1 # 'Brake' the car by counter-throttling
				# self.steering = 0 # Reset steering
				self.brake_data.data = self.prevbrake = self.brake = brake
				self.pub_brake.publish(self.brake_data)
						
						# update previous gear
						# self.throttle_data.data = self.throttle
						# self.pub_throttle.publish(self.throttle_data)
					
			if self.prev_gas != self.throttle:
				self.throttle_data.data = self.throttle = throttle
				self.pub_throttle.publish(self.throttle_data)
				self.prev_gas = self.throttle # update previous throttle"

	# Method Name: corner

	# Input :
	# 1) output: The target coordinate to reach towards with reference to the car frame
	# 2) car_speed: The current speed of the car

	# Output :
	# 1) gear: what type of gear is used in given condition

	# Function Explanation :
	# The codes that will be run when a corner turning is wanted to be performed
	# These includes:
	# Steering control
	# Throttle control
	# Gear control
	def corner(self, output, car_speed):
		rospy.loginfo("Running corner method...")
		gear = "forward"
		if self.goal_type in [8,9,10]: # Reverse into corner
			gear = "reverse"

		# Steering control
		if car_speed > 1 and (self.stop_cal_t <= 10 or (self.stop_cal_t - 10) % 10 == 0): # Settling time = 10 x poll_period
			a = math.atan2(output.pose.position.y,output.pose.position.x)  # alpha
			omega = 1 * a # Scalar constant to define angular velocity omega
			if omega != 0:
				# Apply Ackermann's steering
				r = car_speed / -omega
				self.steering = math.atan(self.b_wheel_base / r)

		if abs(self.steering) > 0.6: # Limit max steering
			self.steering = 0.6*abs(self.steering) / self.steering

		# Throttle control
		if car_speed < 0.5 :  # Low speed condition
			self.throttle = 0.5

		elif not self.stop:  # not at final goal
			self.move = True # Flag when car starts moving
			if car_speed < 2:
				self.throttle = self.fct[0] # Base throttle
			elif self.throttle < 0.49: # Limit max throttle
				self.throttle += self.fct[1] # increment throttle

		else: # End condition
			self.throttle = 0
		

		self.publishThrotBrake(self.throttle)
		rospy.loginfo("Gear = %s, Steering = %f, Throttle = %f", gear, self.steering, self.throttle)
		rospy.loginfo("Exiting corner method...")
		return gear

	def collision_handler(self, msg):
		self.crash = True
		#rospy.loginfo("Collsion detected, COLLISION PROTOCOL starting")


	# Method Name: straight

	# Input :
	# 1) output: The target coordinate to reach towards with reference to the car frame
	# 2) car_speed: The current speed of the car

	# Output :
	# 1) gear: what type of gear is used in given condition

	# Function Explanation :
	# The codes that will be run when a straight car movement is desired
	# These includes:
	# Steering control
	# Throttle control
	# Gear control
	def straight(self, output, car_speed):
		rospy.loginfo("Test 3")
		if self.goal_type == 0:
			rospy.loginfo("move forward in straight")
			gear = "forward"
		elif self.goal_type == 6:
			rospy.loginfo("Test 4")
			rospy.loginfo("move reverse in straight")
			gear = "reverse"

		# Steering control
		if car_speed > 1 and (self.stop_cal_t <= 10 or (self.stop_cal_t - 10) % 10 == 0): # Settling time = 10 x poll_period
			a = math.atan2(output.pose.position.y,output.pose.position.x)  # alpha
			omega = 1 * a # Scalar constant to define angular velocity omega
			if omega != 0:
				# Apply Ackermann's steering
				r = car_speed / -omega
				self.steering = math.atan(self.b_wheel_base / r)

				
		# Throttle control
		if car_speed < 0.5 :  # Low speed condition
			self.throttle = 0.5

		elif not self.stop:  # not at final goal
			self.move = True # Flag when car starts moving
			if car_speed < 2:
				self.throttle = self.fct[0] # Base throttle
			elif self.throttle < 0.49: # Limit max throttle
				self.throttle += self.fct[1] # increment throttle

		else: # End condition
			self.throttle = 0
		if abs(self.steering) > 0.6: # Limit max steering
			self.steering = 0.6*abs(self.steering) / self.steering

		self.publishThrotBrake(self.throttle)
		rospy.loginfo("Gear = %s, Throttle = %f", gear, self.throttle)
		rospy.loginfo("Exiting straight method...")
		return gear

	# Method Name: finalgoal

	# Input :
	# 1) output: The target coordinate to reach towards with reference to the car frame
	# 2) car_speed: The current speed of the car
	# 3) diff_radius: The radial distance between the center of the car and the target coordinate

	# Output :
	# 1) gear: what type of gear is used in given condition

	# Function Explanation :
	# The codes that will be run when the car reach final goal and want to stop and shutdown
	# These includes:
	# Steering control
	# Throttle control
	# Gear control
	def finalgoal(self, output, car_speed, diff_radius):
		gear = self.prev_gear
		rospy.loginfo("Running final method...")		
		# Steering Control
		if car_speed > 1 and (self.stop_cal_t <= 10 or (self.stop_cal_t - 10) % 10 == 0): # Settling time = 10 x poll_period
			a = math.atan2(output.pose.position.y,output.pose.position.x)  # alpha
			omega = 1 * a # Scalar constant to define angular velocity omega
			if omega != 0:
				# Apply Ackermann's steering
				r = car_speed / -omega
				self.steering = math.atan(self.b_wheel_base / r)
		if self.goal_type == 7 and diff_radius < 3: # End condition
			self.stop = True
			self.steering = 0

		# Throttle control
		if car_speed < 0.5 :  # Low speed condition
			self.throttle = 0.5
		else: # End condition
			self.throttle = 0
		if abs(self.steering) > 0.6: # Limit max steering
			self.steering = 0.6*abs(self.steering) / self.steering
		self.publishThrotBrake(self.throttle)
		return gear
	
	# Method Name: odom

	# Input:
	# 1) msg : the odometry data (coordinates in reference to map frame) of current car position  

	# Output :
	# None

	# Function Explanation :
	# Class method that gets called when odometry message is published to /odom by ROS and simulator bridge, and passed to msg variable
	# a) extracts current position and velocity of the car
	# b) extracts the goal coordinates from the input 
	# c) use timer to calculate condition for running callback() method
	# d) update the next goal in the array sequence of input goals only when certain condition is satisfied
	def odom(self, msg):
		if not self.end: # Perform operations while end condition is not true
			# Get cartesian positions
			self.car_x = msg.pose.pose.position.x
			self.car_y = msg.pose.pose.position.y
			self.car_z = msg.pose.pose.position.z
			# Get z-quartenion orientation
			self.yaw = msg.pose.pose.orientation.z
			# Get cartesian velocities
			self.v_x = msg.twist.twist.linear.x
			self.v_y = msg.twist.twist.linear.y
			self.v_z = msg.twist.twist.linear.z

			# Define goal coordinates in map frame (set in msg.header)
			self.goal_map = PoseStamped()
			self.goal_map.header = msg.header
			self.goal_map.pose.position.x = self.goal_x
			self.goal_map.pose.position.y = self.goal_y

			# timer
			t = rospy.get_time() # Get current time in seconds
			if not self.start: # Start condition
				self.start = True
				#rospy.loginfo("Starting control loop(py) with polling period: %.2fs" %(self.poll_period)) # Report set period
				dt = 0
				self.callback(self.t_tot) # Callback at t = 0
			else:
				dt = t - self.t0 # Calculate time elapsed since last time step

			self.t_poll += dt # Total time elapsed from last call to callback function
			self.t_tot += dt # Total time elapsed from start
			self.t0 = t # Update current time for next step

			# Update goal sequence
			if self.arrSize > 0:
				if self.goal_type in [0,5,11]: # Smaller radius for stop & go, and [0,0] goal
					rad = 3
				else:
					rad = 5
				diff = math.sqrt((self.car_x - self.goal_x)**2 + (self.car_y - self.goal_y)**2 + self.car_z**2) # Radial distance to goal
				if diff < rad: # Car reaches within radius defined above
					# Delete goal and its type
					rospy.loginfo('Passed point: [%.2f, %.2f], Goal type: %d', self.pose_seq[0][0], self.pose_seq[0][1], self.pose_types[0])
					self.pose_seq.pop(0)
					self.pose_types.pop(0)
					self.arrSize -= 1
					if self.arrSize > 0:
						# Update goal coordinate and type
						self.goal_x = self.pose_seq[0][0]
						self.goal_y = self.pose_seq[0][1]
						self.goal_type = self.pose_types[0]

			# callback at defined period
			t_err = abs(self.t_poll - self.poll_period)/self.poll_period; # Calculate error
			if (t_err < 0.05 or self.t_poll > self.poll_period): # callback when within 5% precision or when poll elapsed time exceeds defined period
				self.t_poll = 0 # Reset callback elapsed time
				self.callback(self.t_tot)
		else:
			return


	# Method Name: store

	# Input :
	# 1) coord: The target coordinate to reach towards with reference to the map frame
	# 2) types: The type of car movement 

	# Output :
	# None

	# Function Explanation :
	# Use the input type of car movement(types) to move to wanted destination coordinate(coord)
	# Adding new data to 2 arrays, pose_seq and pose_types
	# pose_seq is array for coordinates
	# pose_types is array of goal types 
	def store(self, coord, types):
		self.pose_seq.append(coord)
		self.pose_types.append(types)



	# Method Name: getGoals

	# Input :
	# None

	# Output :
	# None

	# Function Explanation :
	# For user to input goal coordinates and respective goal types by calling the function store 
	def getGoals(self):
		rospy.loginfo("Loading goals for configuration: %d" %(self.config))
        # The goal types are described using numbers and are represented as follow
        # Moving forward
        #   0 - straight goal
        #   1 - goal before corner
        #   2 - corner goal
        #   3 - goal after corner
        #   4 - actual goal in the competition, its control is same as goal type 0 -- removed
        #   5 - 3-point D -- temporarily disabled
        #   7 - last goal, need to fully stop
        # Moving backward
        #   6 - straight goal
        #   8 - goal before corner
        #   9 - corner goal
        #   10 - goal after corner
		#   12 - sharp corner turn -- temporarily disabled
		#   13 - sharp sharp corner turn -- temporarily disabled
		#   14 - sharp sharp sharp corner turn -- temporarily disabled
		#   15 - straight zero throttle -- temporarily disabled
		self.pose_seq =[]
		self.pose_types =[]
		if self.config == 1:	#test cases for corner function
			self.store( [-77.9,-17.59], 0)
			self.store( [-74.8,-13.8], 1)
			self.store( [-71.5,-3.2], 2)
			self.store( [-64.2,-0.8], 3)
			self.store( [-64.2,-0.8], 8)
			self.store( [-71.5,-3.2], 9)
			self.store( [-74.8,-13.8], 10)
			self.store( [-77.9,-17.59], 7)
		if self.config == 2:	#test cases for straight function and final goal function
			self.store( [-77.9,5.59], 0)
			self.store( [-77.9,-7.59], 6)
			self.store( [-77.9,-10.59], 7)
		# Initialize goal coordinates, array size and goal type
		self.arrSize = len(self.pose_types)
		self.goal_x = self.pose_seq[0][0]
		self.goal_y = self.pose_seq[0][1]
		self.goal_type = self.pose_types[0]
		
# Main function
	# Method Name: listener

	# Input :
	# None

	# Output :
	# None

	# Function Explanation :
	# Initialize a ROS node for subscribing and publishing topics
	# Subscribe to wanted ROS topics to collect required data:
	# a) Odometry
	# Keeps the neccessary connection for the the functions above until user cancel it
	# Or there is no neccssary topics or data published by the topics
def listener():
	# Initialize control node
	rospy.init_node('cmd_car_control')

	# Initialize nodelets and get goals
	control = Control()
	rospy.Subscriber("/carla/ego_vehicle/collision", CarlaCollisionEvent, control.collision_handler)
	rospy.Subscriber("/carla/ego_vehicle/odometry", Odometry, control.odom)

	rospy.loginfo("Initialized control node")
	control.getGoals()

	while not rospy.is_shutdown():
		try:
			trans = control.tf2_buffer.lookup_transform("ego_vehicle","map",rospy.Time())
			break
		
		except(tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
			rospy.sleep(0.001)
			continue

		
	# Wait for odom to be available
	rospy.loginfo("Checking for odom...")
	data = rospy.wait_for_message("/carla/ego_vehicle/odometry", Odometry, 20) # Blocks until message is received, 20s timeout
	if data != None:
		rospy.loginfo("Starting control!")
		rospy.spin() # Block the node from exiting
		rospy.loginfo("Ending!!!!!!!!!")
	else:
		rospy.loginfo("No odom data!")





if __name__ == '__main__':
	listener() # Call listener function when script is executed