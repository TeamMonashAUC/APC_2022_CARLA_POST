#!/usr/bin/env python

'''
School: Monash University Malaysia

Written by:     Team Monash SEM Intelligent Department
   			 1) Andrew Joseph Ng Man Loong

Description: - Team Monash Shell Eco Marathon Autonomouse Programming Challenge Research code
   		 
'''

#################################################################################################################################################
# install instruction

# library name - "install command in linux terminal" (link to the library documentation)

'''
simple pid  -  "pip install simple-pid"  (https://pypi.org/project/simple-pid/)
'''


################################################################################################################################################# 
# import other prgramming files

import settings  # settings.py is used to store all global variables between files

import ROS_Communication     # does communications with rostopics & roscore (Level 1 code)
import Movement_Control   	 # utilise PID for throttle & linear steering using maximum turning angle by the car (Level 2 code)

#################################################################################################################################################
# import libraries
import rospy


#################################################################################################################################################
def main():
    Movement_Control.carControl(targetSpeed = 10,steerAngle= 0)

    # ROS_Communication.currentSpeed = Movement_Control.currentSpeed = currentSpeed
    # ROS_Communication.curr_time = Movement_Control.curr_time = curr_time

    # CarlaEgoVehicleInfo.wheels
    # CarlaEgoVehicleInfoWheel.max_steer_angle
    # link = rospy.Subscriber('/carla/ego_vehicle/odometry',Odometry,receive_Odometry)
    # tf2_buffer = tf2_ros.Buffer()
    # goal_map = PoseStamped()

    # goal_map.pose.position.x = 0
    # goal_map.pose.position.y = 0
    # trans = tf2_buffer.lookup_transform("ego_vehicle","map",rospy.Time())
    # output = tf2_buffer.transform(goal_map,"ego_vehicle")
    # rospy.loginfo(output)
# def base(data):
    
    # rospy.loginfo(data.wheels)
    # rospy.loginfo(data.max_steer_angle)






#################################################################################################################################################
try:
    # single time setup

    # start rosnode
    rospy.init_node('APC_Monash')
    rate = rospy.Rate(100) # publish data at 100Hz
    rospy.loginfo("APC_Monash started")

    # start ros communications with rostopics
    ROS_Communication.ROS_Start()

    while not rospy.is_shutdown():
        # infinite loop
        main()
        rate.sleep()

except rospy.ROSInterruptException: # if we stop the script (using CTRL+C), it will run rospy.ROSInterruptException
    
    rospy.loginfo("Exit program successful") # Exit message
    pass
