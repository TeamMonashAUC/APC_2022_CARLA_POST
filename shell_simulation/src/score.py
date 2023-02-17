#!/usr/bin/env python
import rospy
import math
from std_msgs.msg import Int8, Float64,Int8MultiArray
from nav_msgs.msg import Odometry
from carla_msgs.msg import CarlaLaneInvasionEvent, CarlaCollisionEvent

# score.py
# Author(s):  Andrew Joseph

# refferenced work of APC2022: Navaneeth Nair, Lee Khai Hoe, Lee Rui En
# School: Monash University Malaysia
# Description: - Python script that keeps track of goals and penalties while running

goalPassed =0
class Score:
    def __init__(self): # constuctor
        self.cur = Int8() # class variable for passed goals counter
        self.passed = 0 # class variable for current time
        # List of goals
        self.goals = [  [-171.60,4.00,0.00],   #P0
                        [-206.40,4.20,0.00],   #P1
                        [-255.90,0.20,0.00],   #P2
                        [-272.10,-43.90,0.00], #P3
                        [-205.50,-95.00,0.00], #P4
                        [-185.50,-42.40,0.00], #P5
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
                        [74.00,199.20,0.00],   #P18
                        [154.10,177.30,0.00],  #P19
                        [189.20,52.80,0.00],   #P20
                        [174.40,-148.00,0.00], #P21
                        [10.20,-187.90,0.00],  #P22
                        [-145.80,-190.90,8.60],#P23
                        [-232.60,28.10,10.00], #P24
                        [-119.40,186.60,10.00],#P25
                        [84.70,144.10,0.00],   #P26
                        [148.10,112.20,0.00],  #P27
                        [151.40,15.20,0.00],   #28
                        [124.70,1.90,0.00],    #29
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
        self.num_goals = len(self.goals)
        self.pub_c = rospy.Publisher("/Monash/current_score", Int8, queue_size = 10) #initialising publisher to ros

    def odom(self,msg):
        global goalPassed
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
                    goalPassed = self.passed
                    rospy.loginfo("Passed: [%.2f,%.2f]"%(self.goals[i][0],self.goals[i][1])) # logging goals passed message
                    self.goals.pop(i) # remove reached goal from the list array
                    if self.passed == self.num_goals: # if all 12 goals are passed
                        rospy.loginfo("Passed all goals!") # logging all goals passed message
                    rospy.sleep(0.5) # causing ros to sleep for 0.5 milliseconds
                    showStats()
                    return
        else:
            return

        self.cur.data = self.passed
        # publish_msg = Int8MultiArray()
        # publish_msg.data = [self.cur]
        # self.pub_c.publish(publish_msg) # updating ros publisher data
        self.pub_c.publish(self.cur) # updating ros publisher data



###########################################################
laneCrossed = []  # create global array for recording lane crossing events
def LanePenaltyCounter(data):
    global laneCrossed
    # create an array recording all occurence of laneCrossing 
    for x in data.crossed_lane_markings:
        # rospy.loginfo(x)
        laneCrossed.append(x)
    showStats()

collisionCounter=0
def CollisionPenaltyCounter(data):
    global collisionCounter 
    collisionCounter=collisionCounter+1
    showStats()

###########################################################
def showStats():
    rospy.loginfo("")
    
    # lane crossed
    rospy.loginfo("/////////////////////////////")
    rospy.loginfo("dotted line crossed:" + str(laneCrossed.count(1)))
    rospy.loginfo("solid line crossed:" + str(laneCrossed.count(2)))
    rospy.loginfo("double line crossed:" + str(laneCrossed.count(3)))
    rospy.loginfo("Other:" + str(laneCrossed.count(0)))

    # collision
    rospy.loginfo("Collision:" + str(collisionCounter))

    rospy.loginfo("Score:" + str(goalPassed))
    rospy.loginfo("/////////////////////////////")
###########################################################



def listener():
    rospy.init_node('score') # intiialising score node in ros
    score = Score() # initialising object Score
    rospy.Subscriber("/carla/ego_vehicle/odometry", Odometry, score.odom) # Get car position
    rospy.Subscriber("/carla/ego_vehicle/lane_invasion",  CarlaLaneInvasionEvent, LanePenaltyCounter) 
    rospy.Subscriber("/carla/ego_vehicle/collision",  CarlaCollisionEvent, CollisionPenaltyCounter)  
    rospy.spin() # prevents node from exiting

if __name__ == '__main__':
    listener() # executing main function