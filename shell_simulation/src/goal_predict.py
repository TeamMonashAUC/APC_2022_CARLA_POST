
import rospy
import math
import numpy as np

from std_msgs.msg import String, Float64, Float32,Bool
from nav_msgs.msg import Odometry  

from shell_simulation.msg import ActualCoord

def findGoalPointRobust3D(distancesToValidCoordinates):
    goal_predict = []

    possible_coords = np.array([ [-171.60,4.00,0.00],   #P0
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
                        ])
    
    distance_to_possible_points = distances3D(possible_coords) # find the distances to the possible coordinates from the current position
    
    # Assuming that the distance supplied is a normal python list
    for distance in distancesToValidCoordinates:
        possible_distance = distance_to_possible_points[abs(distance_to_possible_points - distance) <= 1]
        if possible_distance.shape[0] != 0:
            goal_predict.append(possible_coords[np.where(distance_to_possible_points == possible_distance)[0], :].tolist())
    return goal_predict

def distances3D(coordinates):

    """Function to find the distances between the current position of the car and a set of coordinates,
     may it be the distances to all the coordinates or the distance to the valid coordinates
     Expect a ndarry for the coordinates"""

    current_pos = np.array([settings.car_coordinate_from_world])
    distances = np.sqrt(np.sum(np.square(coordinates - current_pos), axis=1))
    return distances

if __name__ == '__main__':
    # try:


    # single time setup
    # test()
    # start rosnode
    rospy.init_node('APC_Monash_coord')
    rate = rospy.Rate(100) # publish data at 100Hz
    rospy.loginfo("APC_Monash started")

    # start ros communications with rostopics
    # ROS_Communication.ROS_Start()
    rospy.Subscriber("/score", Score,receive_Score) 
    rospy.Subscriber('/carla/ego_vehicle/odometry',Odometry,receive_Odometry)   


    # global pub_coord_2D
    # global pub_coord_3D 
    # pub_coord_3D = rospy.Publisher("/actual_coord_3D", ActualCoord, queue_size = 10)

    while not rospy.is_shutdown():
        main()

def receive_Odometry(data):
    curr_time = data.header.stamp # obtained from "rosmsg show nav_msgs/Odometry"
    car_coordinate_from_world     = [data.pose.pose.position.x, data.pose.pose.position.y, data.pose.pose.position.z]

    curr_time = data.header.stamp.secs + (data.header.stamp.nsecs)*1e-9

    # settings.car_direction_from_world      = euler_from_quaternion(data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w)
    euler_angle = euler_from_quaternion(data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w)
    # settings.car_direction_from_world      = [euler_angle[0],euler_angle[1],absoluteYaw(euler_angle[2])]
    settings.car_direction_from_world      = [euler_angle[0],euler_angle[1],euler_angle[2]]