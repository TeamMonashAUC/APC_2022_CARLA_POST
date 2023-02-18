#!/usr/bin/env python
from cmath import cos
import rospy
import math
from std_msgs.msg import Float32, Float64, Int8
from nav_msgs.msg import Odometry
from matplotlib import pyplot as plt
from tf.transformations import euler_from_quaternion
from std_srvs.srv import Empty

# efficiency.py
# Author(s): Navaneeth Nair, Khai Hoe
# Documented by: Woon Siang Yi
# School: Monash University Malaysia
# Description: - Python script that calculates distance, duration and energy used by the car to travel from start to finish
#                by subscribing to the car's odometry data, it also calculates the cpu usage of car control and overall cpu usage by
#                subscribing to cpu_monitor
#              - Record the Velocity, Acceleration, Energy, Drag Force and Inertial Force of the car at a defined period
#                and plot several subgraphs consist of these data against time at the end
passed =0
def odom(msg):
	global distance, energy, duration, start
	global x0, y0, z0, v0, t0, end
	global v_x0, v_y0, v_z0

	# Get car position, velocity and orientation
	car_x = msg.pose.pose.position.x
	car_y = msg.pose.pose.position.y
	car_z = msg.pose.pose.position.z
	v_x = msg.twist.twist.linear.x
	v_y = msg.twist.twist.linear.y
	v_z = msg.twist.twist.linear.z
	(roll, pitch, yaw) = euler_from_quaternion([msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w])

	diff_radius = math.sqrt(math.pow(last_goal[0] - car_x,2) + math.pow(last_goal[1] - car_y,2) + car_z*car_z) # calculates the distance between the car and the final goal position using Pythagoras' Theorem
	velocity = math.sqrt(math.pow(v_x, 2) + math.pow(v_y, 2) + math.pow(v_z, 2)) # calculates the resultant velocity of the car

	if diff_radius < 3 and not end: # reached the end goal
		rospy.sleep(1)
		# rospy.loginfo("Results: [Distance(m): %f, Duration(s): %f, Energy(J): %d, cpu_tot(avg): %f, cpu_tot(max): %f, cpu_cc(avg): %f,  cpu_cc(max): %f]" %(distance, duration, math.ceil(energy), cpu_avg, cpu_max, cc_avg, cc_max))

		distance_km = distance/1000
		energy_kWh = energy/3.6e6

		rospy.loginfo("Results:")
		rospy.loginfo("Time(s): %.1f" %( duration))
		rospy.loginfo("Distance(km): %.3f" %(distance_km))
		rospy.loginfo("Energy(kWh): %.3f" %(energy_kWh))
		rospy.loginfo("Efficiency(kWh/km): %.3f" %(energy_kWh / distance_km))

		rospy.loginfo("")
		rospy.loginfo("Penalties Count: %d" %(penaltyCount))
		energy_with_penalty = energy_kWh +  ((energy_kWh *0.02) * penaltyCount) #calculate penalty energy of 2% for every additional rule broken
		rospy.loginfo("Energy with penalties: %.3f" %(energy_with_penalty))
		
		global score
		rospy.loginfo("")
		rospy.loginfo("Efficiency with penalties(kWh/km): %.3f" %(energy_with_penalty/distance_km))
		rospy.loginfo("Goals Passed: %d" %(score))
		




		# rospy.loginfo("Duration(s): %f, Energy(J): %d, cpu_tot(avg): %f, cpu_tot(max): %f, cpu_cc(avg): %f,  cpu_cc(max): %f]" %(distance, duration, math.ceil(energy), cpu_avg, cpu_max, cc_avg, cc_max))
		# rospy.loginfo("Energy(J): %d, cpu_tot(avg): %f, cpu_tot(max): %f, cpu_cc(avg): %f,  cpu_cc(max): %f]" %(distance, duration, math.ceil(energy), cpu_avg, cpu_max, cc_avg, cc_max))
		# rospy.loginfo("cpu_tot(avg): %f, cpu_tot(max): %f, cpu_cc(avg): %f,  cpu_cc(max): %f]" %(distance, duration, math.ceil(energy), cpu_avg, cpu_max, cc_avg, cc_max))
		end = 1

		if showgraph: # plot a graph if enabled
			#plt.figure(figsize=(8, 10))
			fig, ax = plt.subplots(figsize=(8,10))

			plt.subplot(5,1,1)
			plt.plot(t_array,v_array,'-r')
			plt.title("Velocity vs Time")

			plt.subplot(5,1,2)
			plt.plot(t_array,a_array,'-r')
			plt.title("Acceleration vs Time")

			plt.subplot(5,1,3)
			plt.plot(t_array,e_array,'-r')
			plt.title("Energy vs Time")

			plt.subplot(5,1,4)
			plt.plot(t_array,d_array,'-r')
			plt.title("Drag Force vs Time")

			plt.subplot(5,1,5)
			plt.plot(t_array,ma_array,'-r')
			plt.title("Inertial Force vs Time")

			plt.tight_layout()

			for p in fig.axes:
				pos = p.get_position()
				i = 0

				for t in goaltime:
					p.axvline(t,ls ='--',color = 'g') # set line style and color
					p.text(t,pos.x1,i) # add text to the axes
					i += 1
			plt.show()
		return

	elif end:
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
		distance += dd
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
		duration += dt

		# Calculate acceleration
		acceleration = dv/dt

		# Force Calculation
		f_r_x = mass*g*friction*math.cos(pitch) # Road force
		f_r_y = mass*g*math.sin(pitch)
		f_d = 0.5*rho*drag_coeff*area*math.pow(velocity,2) # Drag force
		f_i = mass*acceleration # Inertial force

		# Calculate Energy usage
		f_tot = f_r_x + f_r_y + f_d + f_i # Total force
		e = f_tot*dd
		energy += e

		if showgraph: # update arrays
			v_array.append(velocity)
			t_array.append(duration)
			a_array.append(acceleration)
			d_array.append(f_d)
			ma_array.append(f_i)
			e_array.append(e)
	else:
		t0 = rospy.get_time()
		if velocity > 0.01:
			start = True
score=0
# records the duration to reach each goal in an array
def getscore(num):
	global passed, goaltime,score
	score = num.data
	if showgraph:
		if (passed != num.data):
			goaltime.append(duration)
			passed = num.data

penaltyCount =0
def penalty_calc(data):
	global penaltyCount
	penaltyCount = data.data


def srv_handler(request):
	rospy.loginfo("Distance: %.4f, Energy: %.4f, Time: %.4f", distance, energy, duration)
	return []

def listener():
	global goal_type
	rospy.init_node('efficiency_calc')
	rospy.Subscriber("/carla/ego_vehicle/odometry", Odometry, odom) # Get car position
	rospy.Subscriber("/Monash/current_score", Int8, getscore)
	rospy.Subscriber("/Monash/penalty_score", Int8,penalty_calc)   
	rospy.Service("/disp_results",Empty,srv_handler)
	rospy.spin()


if __name__ == '__main__':
	# Initialize variables
	goals = [[-47.4,-105.9]] # Our common final goals
	last_goal = goals[0] # If using a different final goal, change here
	v0 = 0
	v_x0 = 0
	v_y0 = 0
	v_z0 = 0
	x0 = -77.9
	y0 = -17.59
	z0 = 0.03
	t0 = 0
	distance = 0
	duration = 0
	energy = 0
	end = False
	start = False

	# CPU
	cpu_cnt = 0
	cpu = 0
	cpu_avg = 0
	cpu_max = 0
	cc = 0
	cc_cnt = 0
	cc_avg = 0
	cc_max = 0

	# Energy constants
	mass = 1845.0 # Car mass [kg]
	g = 9.81 # Gravitational constant [m/s^2]
	friction = 0.01 # Rolling friction coefficient
	rho = 1.2 # Air density at NTP [kg/m^3]
	drag_coeff = 0.15000000596046448 # Drag coefficient
	area = 2.22 # Car front chassis area [m^2]

	# Graph
	showgraph = True
	if (showgraph == True):
		v_array = []  # velocity array
		a_array = []  # acceleration array
		e_array = []  # energy array
		d_array = []  # drag force array
		ma_array = []  # inertial force array
		t_array = []  # time array
		goaltime = []
		passed = 0
	listener()

