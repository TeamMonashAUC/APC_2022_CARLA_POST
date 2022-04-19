#!/usr/bin/env python
import rospy
from std_msgs.msg import String, Float64
from nav_msgs.msg import Odometry
from carla_msgs.msg import CarlaEgoVehicleControl


def getOdom(msg):
    global controls, pub
    if gear == "forward":
        controls.reverse = False
    else:
        controls.reverse = True

    controls.throttle = throttle
    controls.steer = -steering

    pub.publish(controls)

def getBrake(brake_data):
    global brake
    brake = brake_data.data

def getGear(gear_data):
    global gear
    gear = gear_data.data

def getThrottle(throttle_data):
    global throttle
    throttle = throttle_data.data

def getSteering(steering_data):
    global steering
    steering = steering_data.data


def listener():
    global controls, pub
    rospy.init_node('tester', anonymous=True)
    pub = rospy.Publisher('/carla/ego_vehicle/vehicle_control_cmd', CarlaEgoVehicleControl, queue_size=1)
    controls = CarlaEgoVehicleControl()
    rospy.Subscriber("/brake_command", Float64, getBrake)  # Get local planner's global plan
    rospy.Subscriber("/gear_command", String, getGear)  # Get local planner's global plan
    rospy.Subscriber("/throttle_command", Float64, getThrottle)  # Get goal
    rospy.Subscriber("/steering_command", Float64, getSteering)  # Get goal
    rospy.Subscriber("/carla/ego_vehicle/odometry", Odometry, getOdom)
    rospy.loginfo("Starting tester node")
    rospy.spin()


if __name__ == '__main__':
    steering = 0
    throttle = 0
    brake = 0
    gear = "forward"
    listener()