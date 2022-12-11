#!/usr/bin/env python
import rospy
import math
from std_msgs.msg import Int8, Float64
from nav_msgs.msg import Odometry

# score.py
# Author(s):  Navaneeth Nair, Lee Khai Hoe
# Documented by: Lee Rui En
# School: Monash University Malaysia
# Description: - Python script that keeps track of goals to reach and update the list of goals when a goal is reached

class Score:
    def __init__(self): # constuctor
        self.cur = Int8() # class variable for passed goals counter
        self.passed = 0 # class variable for current time
        # List of goals
        self.goals = [ [-77.86, 16.80, 0.0],
                        [-15.45, 194.16, 0.0],
                        [-4.32, 110.51, 0.0],
                        [-1.77, -23.78, 0.0],
                        [79.56, -7.79, 0.0],
                        [230.90, -40.58, 0.0],
                        [189.83, -58.67, 0.0],
                        [161.58, -111.42, 0.0],
                        [17.10, -130.70, 0.0],
                        [-9.35, -168.07, 0.0],
                        [-44.25, -193.47, 0.0],
                        [-145.75, -75.70, 0.0],
                        [-145.47, -7.79, 0.0],
                        [-104.58, -0.5, 0.0],
                        [-52.68, -0.91, 0.0] ]
        self.num_goals = len(self.goals)
        self.pub_c = rospy.Publisher("/current_score", Int8, queue_size = 10) #initialising publisher to ros

    def odom(self,msg):
        # Get car position, velocity and orientation
        car_x = msg.pose.pose.position.x
        car_y = msg.pose.pose.position.y
        car_z = msg.pose.pose.position.z
        sizePose = len(self.goals)

        if sizePose > 0: # run only if number of goals is more than 0
            for i in range(sizePose): # iterate through each goals
                # calculating distance between current position and the goals
                diff_radius = math.sqrt(math.pow(self.goals[i][0] - car_x,2) + math.pow(self.goals[i][1] - car_y,2) + math.pow(self.goals[i][2] - car_z,2))
                if diff_radius < 3: # goal is reached if the distance is less than 3 unit
                    self.passed += 1 # incrementing number of goals passed
                    rospy.loginfo("Passed: [%.2f,%.2f]"%(self.goals[i][0],self.goals[i][1])) # logging goals passed message
                    self.goals.pop(i) # remove reached goal from the list array
                    if self.passed == self.num_goals: # if all 12 goals are passed
                        rospy.loginfo("Passed all goals!") # logging all goals passed message
                    rospy.sleep(0.5) # causing ros to sleep for 0.5 milliseconds
                    return
        else:
            return

        self.cur.data = self.passed
        self.pub_c.publish(self.cur) # updating ros publisher data

def listener():
    rospy.init_node('score') # intiialising score node in ros
    score = Score() # initialising object Score
    rospy.Subscriber("/carla/ego_vehicle/odometry", Odometry, score.odom) # Get car position
    rospy.spin() # prevents node from exiting

if __name__ == '__main__':
    listener() # executing main function