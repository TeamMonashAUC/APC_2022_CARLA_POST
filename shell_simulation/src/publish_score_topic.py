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
from tf.transformations import euler_from_quaternion

class PublisherPyNode:
	def __init__(self):
		self.m_publisher = rospy.Publisher("/score", Score, queue_size=10, latch=False)
		self.sum_speed = 0
		self.x0 = -171.60
		self.y0 = 4.0
		self.z0 = 0.2
		self.t0=0
		self.v0=0
		self.v_x0=0
		self.v_y0=0
		self.v_z0=0
		self.duration=0
		self.end=True
		self.start=False
		self.final_goal = []
		# self.msg = Score()
		self.distance_traveled= 0		# distance
		self.energy_spent  = 0			# energy
		self.mean_speed = 0			# average speed    self.sum_speed / count
		self.goal_reached = []			# goals
		self.closest_approach = []		# goal distance
		
		self.time_to_goal = []			# time taken to reach each goal
		self.score = 0					# points passed
		self.mean_cpu_usage = 0			# average cpu usage
		self.penalties = 0				# penalty score
		self.drag_coef=0.15000000596    # Drag coefficient
		
		self.mass=1845.0                # Car mass [kg]
		self.g=9.81                     # Gravitational constant [m/s^2]
		self.rho=1.2                    # Air density at NTP [kg/m^3]
		self.area=2.22                  # Car front chassis area [m^2]
		self.friction=0.01              # Rolling friction coefficient
		
            
		
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
		# self.sample15goals = [  possible_goals[0],
        #                         possible_goals[1],
        #                         possible_goals[2],
        #                         possible_goals[3],
        #                         possible_goals[4],
        #                         possible_goals[5],
        #                         possible_goals[6],
        #                         possible_goals[7],
        #                         possible_goals[8],
        #                         possible_goals[9],
        #                         possible_goals[10],
        #                         possible_goals[11],
        #                         possible_goals[12],
        #                         possible_goals[13],
        #                         possible_goals[14]
		# ]
		self.sample15goals = [  possible_goals[0],
                                possible_goals[1],
                                possible_goals[3],
                                possible_goals[5],
                                possible_goals[10],
                                possible_goals[11],
                                possible_goals[12],
                                possible_goals[14],
                                possible_goals[15],
                                possible_goals[17],
                                possible_goals[18],
                                possible_goals[19],
                                possible_goals[22],
                                possible_goals[26],
                                possible_goals[35]
		]
		self.frame_count = 0
		self.last_goal=self.sample15goals[-1][0:2]
      # Initialise a publisher

        
	# def timerCallback(self, distance, energy, speed, goals, x, y, penalty, count):
	def timerCallback(self, x, y,z, v_x, v_y, v_z):
		msg = Score()
		dd = math.sqrt(math.pow(x - self.x0, 2) + math.pow(y - self.y0, 2) + math.pow(z - self.z0, 2))
		self.x0 = x
		self.y0 = y
		self.z0 = z
    # Initialise an empty message of the custom type

		# Fill in the fields of the message
    
		self.distance_traveled += dd 		# distance
		# self.energy_spent = 0			# energy
		self.mean_speed = 0				# average speed    self.sum_speed / count
		self.goal_reached = []			# goals
		self.closest_approach = []		# goal distance
		
		self.time_to_goal = []			# time taken to reach each goal
		self.score = 0					# points passed
		self.mean_cpu_usage = 0			# average cpu usage
		self.penalties = 0				# penalty score
		




		msg.distance_traveled = self.distance_traveled
		msg.energy_spent = self.energy_spent
		msg.mean_speed = self.mean_speed
		msg.goal_reached = self.goal_reached
		msg.closest_approach = self.closest_approach
		msg.time_to_goal = self.time_to_goal 
		msg.score = self.score
		msg.mean_cpu_usage = self.mean_cpu_usage
		msg.penalties = self.penalties

		# variables for closest distance
		for coor in self.sample15goals:
			# dist_diff = math.sqrt(math.pow(coor[0] - x,2) + math.pow(coor[1] - y,2))
			dist_diff = math.sqrt(math.pow(coor[0] - x,2) + math.pow(coor[1] - y,2)+ math.pow(coor[2] - z,2))
			self.closest_approach.append(dist_diff)
			msg.closest_approach = self.closest_approach



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
		v_x = msg.twist.twist.linear.x
		v_y = msg.twist.twist.linear.y
		v_z = msg.twist.twist.linear.z
		self.timerCallback(car_x, car_y,car_z, v_x, v_y, v_z)
		(roll, pitch, yaw) = euler_from_quaternion([msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w])
		diff_radius=math.sqrt(math.pow(last_goal[0] - car_x,2) + math.pow(last_goal[1] - car_y,2)) # calculates the distance between the car and the final goal position using Pythagoras' Theorem
		velocity=math.sqrt(math.pow(v_x, 2) + math.pow(v_y, 2) + math.pow(v_z, 2)) # calculates the resultant velocity of the car
        # rospy.loginfo("Calculating distance......")
        print("distance travelled: ", self.distance_travelled)
    
        if diff_radius < 3 and end: # reached the end goal
            rospy.sleep(1)
            rospy.loginfo("Results: [Distance(m): %f, Duration(s): %f, Energy(J): %d, cpu_tot(avg): %f, cpu_tot(max): %f, cpu_cc(avg): %f,  cpu_cc(max): %f]" %(distance, duration, math.ceil(energy), cpu_avg, cpu_max, cc_avg, cc_max))
            distance_km = self.distance_travelled/1000
            energy_kWh = self.energy/3.6e6
        


            rospy.loginfo("Results:")
            rospy.loginfo("Time(s): %.1f" %(self.duration))
            rospy.loginfo("Distance(km): %.3f" %(distance_km))
            rospy.loginfo("Energy(kWh): %.3f" %(energy_kWh))
            rospy.loginfo("Efficiency(kWh/km): %.3f" %(energy_kWh / distance_km))

            rospy.loginfo("")
            #rospy.loginfo("Penalties Count: %d" %(penaltyCount))
            #energy_with_penalty = energy_kWh +  ((energy_kWh *0.02) * penaltyCount) #calculate penalty energy of 2% for every additional rule broken
            #rospy.loginfo("Energy with penalties: %.3f" %(energy_with_penalty))
            
            global score
            rospy.loginfo("")
            rospy.loginfo("Efficiency with penalties(kWh/km): %.3f" %(energy_with_penalty/distance_km))
            rospy.loginfo("Goals Passed: %d" %(score))





            # rospy.loginfo("Duration(s): %f, Energy(J): %d, cpu_tot(avg): %f, cpu_tot(max): %f, cpu_cc(avg): %f,  cpu_cc(max): %f]" %(distance, duration, math.ceil(energy), cpu_avg, cpu_max, cc_avg, cc_max))
            # rospy.loginfo("Energy(J): %d, cpu_tot(avg): %f, cpu_tot(max): %f, cpu_cc(avg): %f,  cpu_cc(max): %f]" %(distance, duration, math.ceil(energy), cpu_avg, cpu_max, cc_avg, cc_max))
            # rospy.loginfo("cpu_tot(avg): %f, cpu_tot(max): %f, cpu_cc(avg): %f,  cpu_cc(max): %f]" %(distance, duration, math.ceil(energy), cpu_avg, cpu_max, cc_avg, cc_max))
            end = False

        elif not end:
		    return

        elif start:
            # Calculate velocity and save
            dv = velocity - v0
            v0 = velocity
            # dv = math.sqrt(math.pow(v_x - v_x0, 2) + math.pow(v_y - v_y0, 2) + math.pow(v_z - v_z0, 2))
            v_x0 = v_x
            v_y0 = v_y
            v_z0 = v_z

            # Calculate distance travelled and save
            dd = math.sqrt(math.pow(car_x - x0, 2) + math.pow(car_y - y0, 2) + math.pow(car_z - z0, 2))
            self.distance_travelled += dd
            x0 = car_x
            y0 = car_y
            z0 = car_z

            # Calculate dt and save
            t = rospy.get_time() ## NOTe THAT CARLA is running simulation time and not real time
            dt = t - t0
            # dt = 0.05
            t0 = t
            if dt == 0:
                dt = 0.001
            self.duration += dt

            # Calculate acceleration
            acceleration = dv/dt

            # Force Calculation
            f_r_x = self.mass*self.g*self.friction*math.cos(pitch) # Road force
            f_r_y = self.mass*self.g*math.sin(pitch)
            f_d = 0.5*self.rho*self.drag_coeff*self.area*math.pow(velocity,2) # Drag force
            f_i = self.mass*acceleration # Inertial force

            # Calculate Energy usage
            f_tot = f_r_x + f_r_y + f_d + f_i # Total force
            e = f_tot*dd
            self.energy_spent += e
        else:
            t0 = rospy.get_time()
            print("Velocity change to True")
            if velocity > 0.01:
                start = True


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



