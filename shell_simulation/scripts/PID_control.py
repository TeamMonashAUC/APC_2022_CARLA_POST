# control_master_simple.py
# Author(s): Jayson Teh, Navaneeth Nair, Zi Yu, Khai Hoe
# Documented by: Navaneeth Nair R Dhileepan
# School: Monash University Malaysia
# Description: - Python script that controls the PhysXCar in AirSim from start to finish
#                by subscribing to the car's odometry data, calculating the required throttle,
#                gear & steering values based on pre-defined goal sequences and goal types,
#                and publish these values to AirSim to move the car at a defined period.
#              - This _simple variant simplifies the throttle to basic increments and decrements for low speeds (<= 5m/s)


# required library

'''
pip install simple-pid

'''


#!/usr/bin/env python
import rospy
from std_msgs.msg import String, Float64
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, Point
import math
import tf2_ros
import tf2_geometry_msgs.tf2_geometry_msgs
from carla_msgs.msg import CarlaCollisionEvent
from simple_pid import PID



class Control: # Control class for modular code
	def __init__(self): # Class constructor
		# Constants
		self.fct = [0.35, 0.005, 0.02] # Throttle factors
		self.rad90 = math.radians(90) # 90 degrees in radians
		self.b_wheel_base = 2.9 # Wheelbase length [m]

		# Main parameters
		self.poll_period = 0.2 # Period of calling callback() function
		self.config = 2 # Goal sequence configuration

		# Initialize method attributes (variables global to class)
		self.car_x = self.car_y = self.car_z = self.yaw = self.v_x = self.v_y = self.v_z = self.t_poll = self.t_tot = self.t0 = self.throttle = self.steering = self.prev_gas = self.stop_cal_t = 0
		self.stop = self.end = self.start = self.move = False
		self.prev_gear = "forward"
		self.prev_goal_type = self.goal_type = -1
		self.crash = False
		self.recover = False
		self.cnt = 0
		self.goal15check = False
		self.prevthrottle = 0.5

		# Initialize publishers and messages
		self.pub_gear = rospy.Publisher("/gear_command", String, queue_size = 1)
		self.pub_throttle = rospy.Publisher("/throttle_command", Float64, queue_size = 1)
		self.pub_steering = rospy.Publisher("/steering_command", Float64, queue_size = 1)
		self.throttle_data = self.steering_data = Float64()
		self.gear_data = String()

		# Goal point transformer
		self.goal_map = PoseStamped ()
		self.tf2_buffer = tf2_ros.Buffer()
		self.listener = tf2_ros.TransformListener(self.tf2_buffer)

	# Class method that performs the calculation for controlling the car, total time elapsed from controller start is passed
	def callback(self, time):
		gear = "forward" # Initialize gear to forward
		if self.stop and not self.end: # Exit control when reached final goal
			#rospy.loginfo("Controller Time: %.4f" %(time)) # Display total controller time
			self.end = True
			return
		elif self.end: # End condition
			return

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

		# Steering control
		if car_speed > 1 and (self.stop_cal_t <= 10 or (self.stop_cal_t - 10) % 10 == 0): # Settling time = 10 x poll_period

			a = math.atan2(output.pose.position.y,output.pose.position.x)  # alpha
			omega = 1 * a # Scalar constant to define angular velocity omega
			if self.goal_type == 12:  #sharp corner turn
				omega = 2.5 * a
			elif self.goal_type == 13:  #sharp sharp corner turn
				omega = 5 * a
			elif self.goal_type == 14:  #sharp sharp sharp corner turn
				omega = 8 * a
			if omega != 0:
				# Apply Ackermann's steering
				r = car_speed / -omega
				self.steering = math.atan(self.b_wheel_base / r)

		if self.goal_type in [6,8,9,10,11]: # Reverse goal types
			gear = "reverse"

		if self.goal_type == 7 and diff_radius < 3: # End condition
			self.stop = True
			self.steering = 0

		# Throttle control
		if self.goal15check == True:
			self.throttle = self.prevthrottle
			self.goal15check = False
		if car_speed < 0.5 :  # Low speed condition
			self.throttle = 0.5
		elif not self.stop:  # not at final goal
			self.move = True # Flag when car starts moving
			if self.goal_type not in [5, 11]: # Not at stop & go goal
				if self.goal_type == 15:
					self.goal15check = True
					if self.throttle != 0 and self.prevthrottle != 0:
						self.prevthrottle = self.throttle
					self.throttle = 0 # make throttle 0
				elif car_speed < 2:
					self.throttle = self.fct[0] # Base throttle
				elif self.throttle < 0.49: # Limit max throttle
					self.throttle += self.fct[1] # increment throttle
			else:

				if diff_radius < 20 and self.throttle > 0.1:
					self.throttle -= self.fct[2] # decrement throttle when close to stop & go, min 0.1
		else: # End condition
			self.throttle = 0

		if abs(self.steering) > 0.6: # Limit max steering
			self.steering = 0.6*abs(self.steering) / self.steering
		#rospy.loginfo("Collsion is %s", self.crash)
		if self.crash == True or self.recover == True: ########## PROTOCOLS FOR CRASHING ###########
			self.gear = "reverse"
			self.throttle = 0.5
			self.steering = 0
			self.cnt += 1
			if self.cnt > 10:
				# if self.cnt == 6:
				# 	rospy.sleep(2)
				
				self.crash = True
				self.recover = True
				
				if self.cnt <= 20:
					self.gear = "forward"
					self.throttle = 1
					self.steering = 0
				if self.cnt > 20:
					self.recover = False
					self.crash = False
					self.cnt = 0
			self.pub_gear.publish(self.gear)
			self.pub_throttle.publish(self.throttle)
			self.pub_steering.publish(self.steering)
		### Publish controls ###
		else:
			if not self.move: # Always publish gear & throttle at the start to prevent synching issues
				self.gear_data.data = gear
				self.pub_gear.publish(self.gear_data)
				self.throttle_data.data = self.throttle
				self.pub_throttle.publish(self.throttle_data)

			else: # Publish gear and throttle only during changes
				if gear != self.prev_gear: # Switching between forward and reverse
					# self.throttle = 1 # 'Brake' the car by counter-throttling
					# self.steering = 0 # Reset steering
					self.gear_data.data = gear
					self.pub_gear.publish(self.gear_data)
					self.prev_gear = gear # update previous gear

				if self.prev_gas != self.throttle:
					self.throttle_data.data = self.throttle
					self.pub_throttle.publish(self.throttle_data)
					self.prev_gas = self.throttle # update previous throttle

		if self.stop_cal_t <= 10 or (self.stop_cal_t - 10) % 10 == 0: # Stop publishing steering when reached steady state
			self.steering_data.data = self.steering 
			self.pub_steering.publish(self.steering_data)

		#rospy.loginfo("Publishing: [Throttle:  %f, Brake: %f, Gear: %s, Speed_cur: %f, steer: %f, goal_type: %d, diff_radius: %f, pos_x: %f, pos_y: %f, pos_z: %f, rz: %f]" %(self.throttle, 0,gear,car_speed,self.steering,self.goal_type,diff_radius,self.car_x,self.car_y,self.car_z,self.yaw))

	def collision_handler(self, msg):
		self.crash = True
		#rospy.loginfo("Collsion detected, COLLISION PROTOCOL starting")
	# Class method that gets called when odometry message is published to /odom by the AirSim-ROS wrapper, and passed to msg variable
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
					rospy.loginfo('Passed point: [%.2f,%.2f]',self.pose_seq[0][0],self.pose_seq[0][1])
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

	# Class method that allocates array of coordinates and goal types
	def getGoals(self):
		rospy.loginfo("Loading goals for configuration: %d" %(self.config))
        # The goal types are described using numbers and are represented as follow
        # Moving forward
        #   0 - straight goal
        #   1 - goal before corner
        #   2 - corner goal
        #   3 - goal after corner
        #   4 - actual goal in the competition, its control is same as goal type 0
        #   5 - 3-point D
        #   7 - last goal, need to fully stop
        # Moving backward
        #   6 - straight goal
        #   8 - goal before corner
        #   9 - corner goal
        #   10 - goal after corner
		#   11 - reverse variant of 3-point D

		#   Special add-ons for 2022 APC
		#   12 - sharp corner turn
		#   13 - sharp sharp corner turn
		#   14 - sharp sharp sharp corner turn
		#   15 - straight zero throttle

		# Allocate goals array based on defined config at class declaration
		if self.config == 1:
			# Config 1 - CARLA simple throttle, turn and stop
			self.pose_seq = [[-77.9,-17.59],[-52.68,-0.91]]
			# The points I set- [Origin, Straight goal, straight goal before corner, corner goal, exit corner goal, stop goal]
			self.pose_types = [0,7] 


		elif self.config == 2:
			# Config 2 - Efficiency (< Distance, < Time), (~1520 m, ~162s)
			



			self.pose_seq = [
				[-77.9,-17.59],
				
				[-74.8,-13.8],[-71.5,-3.2],[-64.2,-0.8],
				
				[-52.68,-0.91],  	#1
	
				[-41.4,-0.7],[-29.2,-2.5], [-23.3, -7.6],[-21.7,-11.5], [-16.4, -17.6],[-12.3,-21],[-8.3,-23.1],[-4.1, -23.7],# modified 5
				[-1.77,-23.78], 	#2
				[5.5, -23.3],[8.7, -22.4],[10.8,-20.30],[12.5,-20],[15.7,- 17.1],[18.2,-14.40],[20.5,-11],[22.2, -9.1],[24.1,-8.7],[26.9,-7.79],[30.6,-7.2],[36.9,-7.3], [45.2,-7.4],#modified 6

				[79.56,-7.79],  	#3

				[91.7,-7.79],
				
				[212,-10],[219, -10],[223.1,-10.7],[225.7,-11.9],[227.5,-13.0],[228.2,-13.3],[229.8,-16.0],[230.2,-18.8],[230.5,-25.3],

				[230.9,-40.58],  	#4

				[231,-40.58], [227.3,-52.3],[209.2,-57.2],

				[189.83,-58.67],  	#5

				[180.1,-58.67],[172.4,-63.7],[168.5,-79.8],[166.5,-80.8],[167.3,-88.8],[166.8,-94.3],[165.7, -101.9],[163.2, -107.7],
				
				[161.58,-111.42],	#6
				
				[160.2,-113.2], [156.9,-116.2], [149.2, -123.0] ,[143.6, -125.6],[136.6, -127.6],
				[129.5,-129.2],[111.3,-129.6],[68.6,-130.0],[19.2,-130.7], #modified 7
				[17.1,-130.7],  	#7

				# [7.9,-130.4],[-1.9,-134.3],[-9,-146.4], 
				#### Please check this line 
				[8.1,-132.3], [1.0,-135.0], [-6.9,-141.2], [-8.7, -145.4],[-8.8, -151.2],[-9.2, -155.3],

				[-9.0,-191.3], #8
				[-16.2,-193.8], [-29.3,-193.6],

				[-36.9,-193.6], #9

				[-44.2,-193.2], [-51.1,-192], [-56.1,-189.8], [-63.2,-185.0], [-67.2,-179.8], [-70.0,-175.4], [-72.3,-171.1], [-73.8, -165.2],[-74.4, -160.7],[-74.4,-147.5],

				[-74.4,-145.1], [-78.5,-138.7], [-85.4, -135.4],[-90.2,-133.8],[-93.0,-133.2],[-96.6, -132.8],[-104.4,-132.8],   	[-110.5,-133.0],[-119.4,-133.3],[-122.8,-133.4],[-124.9,-133.3],[-126.7,-133.0],[-130.3,-131.9],[-133.7, -130.2],[-136.0,-128.5],[-138.0,-126.6],[-140.2,-123.5],[-142.8,-119.6],[-143.6, -117.6],[-144.7, -111.1],[-145.4, -108.3], [-145.5, -103.7], [-145.5, -100.4],[-145.5, -92,5],[-145.75, -78.7],

				[-145.75,-75.7],	#10
				[-145.4,-51.5],[-145.4,-39.5],[-145.4,-23.2],

				[-145.47,-7.79],    #11
				[-145.47, -5.7],[-144.4,-3.5], [-142.7, -2.3], [-141.5, -1.7], [-140.1,-1.1],[-138.0,-0.5],[-135.8,-0.4],[-133.0, -0.3],[-131.2,-0.4],[-127.8,-0.5],#modified 8
				[-104.58,-0.5], 	#12

				[-101.1,-0.6],[-84.9,1.7],[-77.8,11.5], 
				[-77.86,16.80], 	#13

				[-75,40],[-75,115],
				[-73.5,170.8], [-72.8, 178.9],[-67.9,187.3],[-47.8,194.9],
					
				[-15.45,194.16],	#14
				[-9.7, 194.2],[-4.4, 190.9],[-3.2, 187.7], [-3.2, 182.4], [-3.5, 172.6],[-4.4, 124.7],#modified 9

				[-4.32,110.51] 		#15
				
			]
			
			self.pose_types = [
				0,
				
				1,2,3,

				4, 		#1

				1,2,12,12,12,12,12,12, #modified 5
				12, 		#2
				12,12,12,12,12,12,12,12,12,2,12,3,0,

				0, 		#3

				1,
				
				1,1,12,13,13,13,12,13,3,   #modified 3

				4, 		#4

				1,2,3,

				4, 		#5

				1,2,3,15,13,12,12,2,

				12, 	#6

				2,2,2, 2, 2, 
				2,3,0,0,#modified 7
				2, 		#7

				# 1,13,3,
				##### Please check this line and line 334
				12,13,15,13,12,3,

				############################ ORIGINAL CODE 
				#12, 		#8
				#12,14,13,13,12,3,  #modified 1

				#4, 		#9

				# 1,2,2,2,3,
				# 1,2,3,  1,2,3,
				########################### ORIGINAL CODE (END)

				########################### AMMENDED VERSION 
				1,			#8 
				14,3,

				4,			#9
				
				1,2,12,12,12,15,12,12,2,3,
				13,13,15,13,15,3,3,  2,15,15,15,2,2,2,2,1,15,2,2,1,15,2,1,15,2,
				########################### UP TO THIS PART ^ 
				
				2,		#10
				2,3,15,

				12,		#11
				14, 15, 14, 15, 14, 15, 14, 15,2, 3,  #modified 4 && 8

				4, 			#12

				1,2,3,
				4, 		#13  
				
				0,0,
				1,2,4,3,
				
				4,		#14 #modified 2
				13, 13, 13, 4, 3, 0,	#modified 9
				
				7
			] # 1-11



		# Initialize goal coordinates, array size and goal type
		self.arrSize = len(self.pose_types)
		self.goal_x = self.pose_seq[0][0]
		self.goal_y = self.pose_seq[0][1]
		self.goal_type = self.pose_types[0]

# Main function
def listener():

	# Ros name (allow ros to identify this code as ...)
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
	else:
		rospy.loginfo("No odom data!")

if __name__ == '__main__':
	listener() # Call listener function when script is executed