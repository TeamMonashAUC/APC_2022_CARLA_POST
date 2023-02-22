#!/usr/bin/env python
import rospy
import math
from std_msgs.msg import Int8, Float64, uint8
from nav_msgs.msg import Odometry
from carla_msgs.msg import CarlaLaneInvasionEvent, CarlaCollisionEvent

# score.py
# Author(s):  Andrew Joseph Ng
# Documented by: 
# School: Monash University Malaysia
# Description: - Python script that keeps track of penalties occured

laneCrossed = []  # create global array for recording lane crossing events
def LanePenaltyCounter(data):
    global laneCrossed
    # create an array recording all occurence of laneCrossing 
    for x in data.crossed_lane_markings:
        # rospy.loginfo(x)
        laneCrossed.append(x)

    showStats()
    # rospy.loginfo("laneCrossed: "+ str(laneCrossed))

    # rospy.loginfo("")
    # rospy.loginfo("dotted line crossed:" + str(laneCrossed.count(1)))
    # rospy.loginfo("solid line crossed:" + str(laneCrossed.count(2)))
    # rospy.loginfo("double line crossed:" + str(laneCrossed.count(3)))
    # rospy.loginfo("Other:" + str(laneCrossed.count(0)))

collisionCounter=0
def CollisionPenaltyCounter(data):
    global collisionCounter 
    # rospy.loginfo(data)
    collisionCounter=collisionCounter+1
    showStats()

def showStats():
    rospy.loginfo("")
    
    # lane crossed
    rospy.loginfo("dotted line crossed:" + str(laneCrossed.count(1)))
    rospy.loginfo("solid line crossed:" + str(laneCrossed.count(2)))
    rospy.loginfo("double line crossed:" + str(laneCrossed.count(3)))
    rospy.loginfo("Other:" + str(laneCrossed.count(0)))

    # collision
    rospy.loginfo(collisionCounter)

def listener():
    rospy.init_node('score') # intiialising score node in ros
    rospy.Subscriber("/carla/ego_vehicle/lane_invasion",  CarlaLaneInvasionEvent, LanePenaltyCounter) 
    rospy.Subscriber("/carla/ego_vehicle/collision",  CarlaCollisionEvent, CollisionPenaltyCounter)  

    rospy.spin() # prevents node from exiting

if __name__ == '__main__':
    listener() # executing main function