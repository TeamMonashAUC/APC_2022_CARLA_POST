#!/usr/bin/env python

import rospy
import math
from std_msgs.msg import Float32, Float64, Int8, Int8MultiArray
from nav_msgs.msg import Odometry
from matplotlib import pyplot as plt
from tf.transformations import euler_from_quaternion
from std_srvs.srv import Empty
from cmath import cos
from shell_simulation.msg import Score
from std_msgs.msg import Int8, Float64,Int8MultiArray,Float32
from nav_msgs.msg import Odometry
from carla_msgs.msg import CarlaLaneInvasionEvent, CarlaCollisionEvent

class PublisherPyNode:
	def __init__(self):
		self.m_publisher = rospy.Publisher("/Score", Score, queue_size=10, latch=False)
		self.sum_speed = 0

		# sample for normal use with coordinates directly
		'''
		# self.sample15goals = [  [-171.60,4.00,0.00],
        #                         [-206.40,4.20,0.00],
        #                         [-255.90,0.20,0.00],
        #                         [-272.10,-43.90,0.00],
        #                         [-205.50,-95.00,0.00],
        #                         [-185.50,-42.40,0.00], 
        #                         [-151.10,-151.00,0.00], 
        #                         [-101.40,-154.70,0.00], 
        #                         [-47.80,-117.20,0.00], 
        #                         [-43.80,-56.80,0.00], 
        #                         [-43.90,-17.10,0.00], 
        #                         [3.00,-2.70,0.00], 
        #                         [47.80,-1.80,0.00], 
        #                         [89.00,-5.50,0.00], 
        #                         [45.90,-84.90,0.00]]
		'''

		# listing out possible coordinates, and then indexing them to easily test different paths
		possible_goals =[ [-171.60,4.00,0.00],   #P0
                        [-206.40,4.20,0.00],   #P1
                        [-255.90,0.20,0.00],   #P2
                        [-272.10,-43.90,0.00], #P3
                        [-205.50,-95.00,0.00], #P4
                        [-185.50,-142.40,0.00], #P5 #changed
                        [-151.10,-151.00,0.00],#P6
                        [-101.40,-154.70,0.00],#P7
                        [-47.80,-117.20,0.00], #P8
                        [-43.80,-56.80,0.00],  #P9
                        [-43.90,-17.10,0.00],  #P10
                        [3.00,-2.70,0.00],     #P11
                        [47.80,-1.80,0.00],    #P12
                        [89.00,-5.50,0.00],    #P13
                        [45.90,-84.90,0.00],   #P14
                        [31.30,19.30,0.00],    #P15
                        [36.30,67.20,0.00],    #P16
                        [38.60,155.10,0.00],   #P17
                        [74.00,190.20,0.00],   #P18 # changed
                        [154.10,177.30,0.00],  #P19
                        [189.20,52.80,0.00],   #P20
                        [174.40,-148.00,0.00], #P21
                        [10.20,-187.90,0.00],  #P22
                        [-145.80,-190.90,8.60],#P23
                        [-232.60,28.10,10.00], #P24
                        [-119.40,186.60,10.00],#P25
                        [84.70,144.10,0.00],   #P26
                        [148.10,112.20,0.00],  #P27
                        [151.40,15.20,0.00],   #P28
                        [124.70,1.90,0.00],    #P29
                        [96.20,-28.60,0.00],   #P30
                        [-9.50,-88.30,0.00],   #P31
                        [-83.20,-87.70,0.00],  #P32
                        [-124.30,-42.40,0.00], #P33
                        [-121.80,28.10,0.00],  #P34
                        [-124.40,106.30,0.00], #P35
                        [-80.20,133.30,0.00],  #P36
                        [-20.70,87.90,0.00],   #P37
                        [25.70,65.40,0.00],    #P38
                        [24.60,-30.70,0.00]    #P39
						]
		

		# allow us to select which coordinate at which points using index
		self.sample15goals = [  possible_goals[0],
                                possible_goals[0],
                                possible_goals[0],
                                possible_goals[0],
                                possible_goals[0],
                                possible_goals[0],
                                possible_goals[0],
                                possible_goals[0],
                                possible_goals[0],
                                possible_goals[0],
                                possible_goals[0],
                                possible_goals[0],
                                possible_goals[0],
                                possible_goals[0],
                                possible_goals[0]
		]
		self.frame_count = 0
      # Initialise a publisher

        
	# def timerCallback(self, distance, energy, speed, goals, x, y, penalty, count):
	def timerCallback(self, x, y):
		msg = Score()
    # Initialise an empty message of the custom type

		# Fill in the fields of the message
		msg.distance_traveled = 0		# distance
		msg.energy_spent = 0			# energy
		msg.mean_speed = 0				# average speed    self.sum_speed / count
		msg.goal_reached = []			# goals
		msg.closest_approach = []		# goal distance
		
		msg.time_to_goal = []			# time taken to reach each goal
		msg.score = 0					# points passed
		msg.mean_cpu_usage = 0			# average cpu usage
		msg.penalties = 0				# penalty score


		# variables for closest distance
		for coor in self.sample15goals:
			dist_diff = math.sqrt(math.pow(coor[0] - x,2) + math.pow(coor[1] - y,2))
			msg.closest_approach.append(dist_diff)



		# publish to rostopic 		
		self.m_publisher.publish(msg)   
	# global m_publisher
	# m_publisher = rospy.Publisher("/Score", Score, queue_size=10, latch=False)

	# frame_count = 0
	# rospy.loginfo("frame_count:", frame_count)
    # frame_count += 1
    # PublisherPyNode.timerCallback(distance_km,energy_kWh,velocity,score,car_x,car_y, penaltyCount, frame_count)
    # Publish a message
	def odom(self, msg):
		car_x = msg.pose.pose.position.x
		car_y = msg.pose.pose.position.y
		car_z = msg.pose.pose.position.z
		self.timerCallback(car_x, car_y)

def listener():	
	node = PublisherPyNode()
	rospy.Subscriber("/carla/ego_vehicle/odometry", Odometry, node.odom)
		# rate = rospy.Rate(100) # publish data at 100Hz
		# while not rospy.is_shutdown():
		# rospy.Subscriber("/carla/ego_vehicle/lane_invasion",  CarlaLaneInvasionEvent, LanePenaltyCounter) 
        
        
        # rospy.Subscriber("/carla/ego_vehicle/collision",  CarlaCollisionEvent, CollisionPenaltyCounter)  
        # rospy.Subscriber('/carla/ego_vehicle/speedometer',Float32,receive_Speedometer)   

        # global publish_penalty_score
        # publish_penalty_score = rospy.Publisher("/Monash/penalty_score", Int8, queue_size = 10) #initialising publisher to ros
	rospy.spin() # prevents node from exiting

if __name__ == '__main__':
	try:
		rospy.init_node('score_node')
		# print("xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxtestxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx")
		# rospy.loginfo("xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxtestxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx")
		listener() # executing main function
	except rospy.ROSInterruptException:
		pass



