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
		self.sample15goals = [[-171.60,4.00,0.00],
                                [-206.40,4.20,0.00],
                                [-255.90,0.20,0.00],
                                [-272.10,-43.90,0.00],
                                [-205.50,-95.00,0.00],
                                [-185.50,-42.40,0.00], 
                                [-151.10,-151.00,0.00], 
                                [-101.40,-154.70,0.00], 
                                [-47.80,-117.20,0.00], 
                                [-43.80,-56.80,0.00], 
                                [-43.90,-17.10,0.00], 
                                [3.00,-2.70,0.00], 
                                [47.80,-1.80,0.00], 
                                [89.00,-5.50,0.00], 
                                [45.90,-84.90,0.00]]
		self.frame_count = 0
      # Initialise a publisher

        
	# def timerCallback(self, distance, energy, speed, goals, x, y, penalty, count):
	def timerCallback(self, x, y):
		msg = Score()
    # Initialise an empty message of the custom type

		# Fill in the fields of the message
		msg.distance_traveled = 0#distance
		msg.energy_spent = 0#energy
		# self.sum_speed += speed
		msg.mean_speed = 0#self.sum_speed / count
		msg.goal_reached = []#goals
		msg.closest_approach = []
		for coor in self.sample15goals:
			dist_diff = math.sqrt(math.pow(coor[0] - x,2) + math.pow(coor[1] - y,2))
			msg.closest_approach.append(dist_diff)
		msg.time_to_goal = []
		msg.score = 0
		msg.mean_cpu_usage = 0
		msg.penalties = 0#penalty
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



