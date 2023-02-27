#!/usr/bin/env python

'''
School: Monash University Malaysia

Written by:     Team Monash SEM Intelligent Department
   			 1) Andrew Joseph Ng Man Loong
             2) Nulan Dammage 
             3) Lucas Wee
             4) Derrick Teoh
             5) Lai Qi Shiun


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

import shell_simulation_2.settings as settings # settings.py is used to store all global variables between files
# from shell_simulation.ROS_Communication import test
import shell_simulation_2.ROS_Communication as ROS_Communication  # does communications with rostopics & roscore (Level 1 code)
import shell_simulation_2.Movement_Control as Movement_Control   	 # utilise PID for throttle & linear steering using maximum turning angle by the car (Level 2 code)
import shell_simulation_2.Coordinate_System as Coordinate_System  	 # move car to coordinate points on the map (Level 3 code)

#################################################################################################################################################
# import libraries
import rospy
import math
import numpy as np

#################################################################################################################################################

def R1(valid_coordinates):
    Coordinate_System.travel_to(40, [-206.4, 4.2]) # P1
    rospy.loginfo(f"{Coordinate_System.findGoalPoint(Coordinate_System.distances(valid_coordinates))}")

    Coordinate_System.travel_to(40, [-245.5,0.8]) 
    Coordinate_System.travel_to(30, [-255.9,0.2]) # P2   #stop

    Coordinate_System.corner(20,8,180,[-272.5,-18.7],-90) #turn left
    Coordinate_System.travel_to(20, [-272.1,-43.9]) # P3
    Coordinate_System.travel_to(15, [-272.0,-68.2]) 

    # curve
    Coordinate_System.travel_to(15, [-271.3,-75.5]) 
    Coordinate_System.travel_to(15, [-268.2,-80.7]) 
    Coordinate_System.travel_to(15, [-264.6,-84.3]) 
    Coordinate_System.travel_to(15, [-253.8,-89.6]) 
    Coordinate_System.travel_to(25, [-241.8,-91.2]) 

    Coordinate_System.travel_to(25, [-219.0,-94.6]) # P4
    Coordinate_System.travel_to(25, [-205.5,-95.0]) # P4
    Coordinate_System.travel_to(25, [-205.3,-94.7]) # traffic light 

def R2(valid_coordinates):
    # curve
    Coordinate_System.travel_to(15, [-199.1,-95.0])
    Coordinate_System.travel_to(20, [-195.0,-104.6]) 
    Coordinate_System.travel_to(20, [-195.0,-118.3]) 
    Coordinate_System.travel_to(20, [-193.7,-127.2]) 
    Coordinate_System.travel_to(20, [-191.8,-132.4]) 
    Coordinate_System.travel_to(15, [-183.9,-144.2]) 
    Coordinate_System.travel_to(10, [-175.5,-150.1]) 
    # Coordinate_System.travel_to(15, [-165.0,-153.9]) 

    Coordinate_System.travel_to(15, [-158.0,-151.1]) 
    Coordinate_System.travel_to(25, [-151.0,-151.0]) #P6 
    Coordinate_System.travel_to(35, [-145.6,-151.0]) # traffic light 


    Coordinate_System.travel_to(25, [-121.0,-151.1]) 
    Coordinate_System.travel_to(20, [-101.4,-154.7]) # P7

    #curve
    Coordinate_System.travel_to(15, [-85.8,-154.9]) 
    Coordinate_System.travel_to(15, [-75.0,-152.7]) 
    Coordinate_System.travel_to(15, [-65.8,-148.7]) 
    Coordinate_System.travel_to(15, [-54.4,-136.6]) 

    Coordinate_System.travel_to(20, [-47.8,-117.2]) # P8 
    Coordinate_System.travel_to(20, [-47.4,-105.9]) 

def R3():
    Coordinate_System.travel_to(20, [-83.20,-87.70]) #P32
    Coordinate_System.travel_to(30, [-105.9,-87.8]) 
    
    Coordinate_System.corner(15,5,180,[-124.1,-72.3],90) #turn right
    Coordinate_System.travel_to(30, [-124.30,-42.40]) # P33
    Coordinate_System.travel_to(20, [-124.50,-17.5])
    Coordinate_System.travel_to(40, [-124.50,73.0])  
    
def R4():
    # [-80.20,133.30,0.00],  #P36
    Coordinate_System.travel_to(40, [-124.4,106.3])
    Coordinate_System.travel_to(30, [-124.4,123.3]) #P35
    Coordinate_System.travel_to(20, [-123.9,136.8])
    Coordinate_System.travel_to(15, [-107.6,138.7])
    Coordinate_System.travel_to(20, [-96.6,138.6])
    Coordinate_System.travel_to(20, [-82.6,134.5])
    Coordinate_System.travel_to(20, [-80.2,133.3]) #P36
    Coordinate_System.travel_to(20, [-75.7,131.8])
    Coordinate_System.travel_to(20, [-75.0,131.4])
    Coordinate_System.travel_to(20, [-71.7,129.6])
    Coordinate_System.travel_to(20, [-66.0,125.1])
    Coordinate_System.travel_to(20, [-60.5,118.5])
    Coordinate_System.travel_to(20, [-58.4,114.9])
    Coordinate_System.travel_to(20, [-55.7,106.1])
    Coordinate_System.travel_to(20, [-55.5,104.7])
    Coordinate_System.travel_to(20, [-54.8,97.8])
    Coordinate_System.travel_to(15, [-35.6,87.8])

    


def R5():
    # "spawn_point": {"x": -124.9, "y": 55.5, "z": 0.2, "roll": 0.0, "pitch": 0.0, "yaw": 90.0},
    # Coordinate_System.corner(15,5,90,[-106.2,88.1],0) #turn right
    # Coordinate_System.travel_to(30, [-65.8,87.9]) 
    # Coordinate_System.travel_to(25, [-33.5,87.9]) 
    Coordinate_System.travel_to(20, [-20.70,87.90]) # P37
    Coordinate_System.travel_to(15, [16,87.8]) 

def R6():
    Coordinate_System.travel_to(15, [47.9,146.0]) 
    Coordinate_System.travel_to(15, [84.70,144.10]) #P26

    #curve
    Coordinate_System.travel_to(15, [94,142.7]) 
    Coordinate_System.travel_to(15, [103.8,141.1]) 
    Coordinate_System.travel_to(15, [114.3,138.8]) 
    Coordinate_System.travel_to(15, [124.4,134.7]) 
    Coordinate_System.travel_to(15, [132.8,129.8]) 
    Coordinate_System.travel_to(15, [138.6,125.0]) 
    Coordinate_System.travel_to(15, [143.1,119.7]) 
    Coordinate_System.travel_to(15, [146.1,115.1]) 
    Coordinate_System.travel_to(15, [148.10,112.20]) #P27 
    Coordinate_System.travel_to(15, [150.0,106.60]) 
    Coordinate_System.travel_to(15, [150.8,101.40]) 
    Coordinate_System.travel_to(15, [151.3,94.4]) 
    Coordinate_System.travel_to(15, [151.4,87.1]) 

    Coordinate_System.travel_to(15, [151.4,13.2]) 
    Coordinate_System.corner(15,5,-90,[141.4,2.2],-180) #turn right
    Coordinate_System.travel_to(15, [124.70,1.9]) #P29
    Coordinate_System.travel_to(30, [116.5,2.2]) 

def R7():
    Coordinate_System.travel_to(15, [96.2,-14.9])
    Coordinate_System.travel_to(15, [96.20,-28.60]) #P30

    #curve
    Coordinate_System.travel_to(15, [95.70,-43.60]) 
    Coordinate_System.travel_to(15, [95,-52]) 
    Coordinate_System.travel_to(15, [92.8,-59.7]) 
    Coordinate_System.travel_to(15, [88.8,-67.3]) 
    Coordinate_System.travel_to(15, [83.4,-73.5]) 
    Coordinate_System.travel_to(15, [76.3,-79]) 
    Coordinate_System.travel_to(15, [65.9,-83.4]) 
    Coordinate_System.travel_to(15, [55.2,-84.9]) 

    Coordinate_System.travel_to(15, [45.90,-84.90]) #P30
    Coordinate_System.travel_to(15, [45.30,-84.90]) 

def R8(): 
    Coordinate_System.travel_to(30, [-32.8,-88.0]) #P31

def R9():
    Coordinate_System.travel_to(30, [-43.7,-73.9]) 
    Coordinate_System.travel_to(25, [-43.80,-56.80]) #P9
    Coordinate_System.travel_to(20, [-43.5,-16.9]) #P10

    Coordinate_System.corner(15,5,90,[-33.9,-2.6],0) #turn right
    Coordinate_System.travel_to(15, [3.00,-2.70]) #P11
    Coordinate_System.travel_to(15, [14.40,-2.60]) 
    

def R10():
    Coordinate_System.travel_to(15, [24.8,-19.3]) 
    Coordinate_System.travel_to(15, [24.60,-30.70]) #P39
    Coordinate_System.travel_to(15, [24.4,-72.3]) 


def R11():
    Coordinate_System.travel_to(15, [44.1,-1.9]) 
    Coordinate_System.travel_to(15, [47.80,-1.80]) #P12
    Coordinate_System.travel_to(15, [66.4,-5.2]) 
    Coordinate_System.travel_to(15, [89.00,-5.50]) #P13

def R12():
    Coordinate_System.travel_to(15, [25.70,65.40]) #P38
    Coordinate_System.travel_to(15, [27.8,15.8]) 

def R13():

    Coordinate_System.travel_to(30, [31.6,15.4]) 
    Coordinate_System.travel_to(20, [31.30,19.30]) #P15

    Coordinate_System.travel_to(20, [35.4,46.2]) 
    Coordinate_System.travel_to(25, [36.30,67.2]) #P16
    Coordinate_System.travel_to(25, [36.5,74.5]) 

def R14():
    pass


def main():
    # reverse for first coordinate 

    # the valid coordinates are known.
    valid_coordinates = Coordinate_System.generate_random_coordinates()
    #valid_coordinates = np.array([[-47.8, -117.2, 0]])
    goal_predict = []

    diff_goal = 3
    while not rospy.is_shutdown():
        rospy.ROSInterruptException  # allow control+C to exit the program        
        Movement_Control.carControl(targetSpeed = -10,steerAngle = 0)
        rate.sleep()
        goal_coord_from_car = Coordinate_System.goal_position_from_car([-171.60,4.00])
        diff_goal = math.sqrt(goal_coord_from_car[0]**2 + goal_coord_from_car[1]**2)

        if(diff_goal<2.5):
            break


    # provided array is in settings.coord_distance
    rospy.loginfo(np.array(settings.coord_distance))

    # valid_coordinates = Coordinate_System.generate_random_coordinates()
    # valid_distance = Coordinate_System.distances(valid_coordinates)
    valid_distance = np.array(settings.coord_distance)

    rospy.loginfo(valid_distance)

    rospy.loginfo("")
    rospy.loginfo("actual coord: ")
    Coordinate_System.findGoalPoint(valid_distance, goal_predict)
    rospy.loginfo(f"The goal coordinates {goal_predict}")
    
    '''
    R1()
   
    R2()
    
    Coordinate_System.corner(15,6,90,[-65.8,-87.9],0) #turn left

    rospy.loginfo(f"The valid coordinates: {valid_coordinates}")
    quit()
    R3()
    
    R4()

    R5()

    Coordinate_System.corner(15,5,0,[29.4,75.5],-90) #turn right
    R12()
    R10()
    Coordinate_System.corner(15,5,-90,[14.7,-88],-180) #turn right
    R8()
    Coordinate_System.corner(15,5,-180,[-43.7,-73.9],90) #turn right
    R9()
    R11()
    Coordinate_System.corner(15,5,0,[96.2,-14.9],-90) #turn right
    R7()
    Coordinate_System.corner(15,5,-180,[31.7,-68.7],90) #turn right
    Coordinate_System.travel_to(15, [31.7,-16.2]) 
    R13()

    Coordinate_System.travel_to(15, [37.2,105.6]) 
    Coordinate_System.travel_to(15, [38,132.2]) 
    Coordinate_System.corner(15,5,90,[47.9,146.0],0) #turn right
                                                                                                                                                                                                                                                                      
    R6()

    Coordinate_System.travel_to(15, [87.6,5.1]) 
    Coordinate_System.travel_to(15, [44.7,5.2]) 
    Coordinate_System.corner(15,5,180,[31.6,15.4],90) #turn right

    R13()
    Coordinate_System.travel_to(15, [37.2,105.6]) 
    Coordinate_System.travel_to(25, [38,132.2]) 
    Coordinate_System.travel_to(30, [39.1,172.8]) 

    # R14()
    
    
    #Final ring
    Coordinate_System.travel_to(10, [39.1,176.1])
    # Coordinate_System.corner(10,5,90[59.0,190.0],0)
    Coordinate_System.travel_to(10, [39.6,188.5])
    Coordinate_System.travel_to(10, [44.0,189.4])
    # Coordinate_System.travel_to(30, [50.0,190.0])
    Coordinate_System.travel_to(15, [50.0,190.2])
    Coordinate_System.travel_to(20, [52.0,190.5])
    Coordinate_System.travel_to(20, [54.0,190.5]) 
    Coordinate_System.travel_to(30, [74.0,190.5])   #P18 (74.0, 190.0)

    # Cutting to the side lane 
    Coordinate_System.travel_to(15, [95.1,193.7])
    Coordinate_System.travel_to(15, [95.5,193.7])
    Coordinate_System.travel_to(15, [108.0,193.7])
    Coordinate_System.travel_to(15, [115.8,192.5])
    Coordinate_System.travel_to(15, [122.0,191.5])
    Coordinate_System.travel_to(15, [126.0,190.5])
    Coordinate_System.travel_to(15, [131.5,188.8])
    Coordinate_System.travel_to(15, [135.0,187.5])
    Coordinate_System.travel_to(15, [140.0,185.5])
    Coordinate_System.travel_to(15, [145.7,182.2])
    Coordinate_System.travel_to(15, [148.1,181.0])  
    Coordinate_System.travel_to(15, [154.1,177.3])  #P19
    Coordinate_System.travel_to(15, [165.0,168.0])
    Coordinate_System.travel_to(15, [174.5,158.0])
    Coordinate_System.travel_to(15, [180.2,150.6])
    Coordinate_System.travel_to(15, [184.1,143.5])
    Coordinate_System.travel_to(15, [187.5,137.1])
    Coordinate_System.travel_to(15, [190.0,130.7])
    Coordinate_System.travel_to(15, [192.5,121.4])
    Coordinate_System.travel_to(15, [193.7,117.3])
    Coordinate_System.travel_to(15, [195.6,101.3])
    Coordinate_System.travel_to(15, [189.1,72.0])   #189.3?
    Coordinate_System.travel_to(15, [189.2,52.8])   #P20
    Coordinate_System.travel_to(20, [189.2,50.0])
    Coordinate_System.travel_to(20, [190.0,-4.0])
    Coordinate_System.travel_to(20, [189.5,-80.0])
    Coordinate_System.travel_to(30, [188.9,-104.7])
    Coordinate_System.travel_to(20, [187.4,-116.4])
    Coordinate_System.travel_to(20, [183.9,-129.5])
    Coordinate_System.travel_to(20, [181.5,-135.7])
    Coordinate_System.travel_to(15, [179.0,-140.0])
    Coordinate_System.travel_to(15, [175.5,-146.0])
    Coordinate_System.travel_to(15, [174.4,-148.0]) #P21
    Coordinate_System.travel_to(15, [168.5,-156.0])
    Coordinate_System.travel_to(15, [156.6,-168.0])
    Coordinate_System.travel_to(15, [143.5,-176.9])
    Coordinate_System.travel_to(15, [134.7,-180.9])
    Coordinate_System.travel_to(15, [123.1,-185.0])
    Coordinate_System.travel_to(15, [111.1,-187.4])
    Coordinate_System.travel_to(15, [104.2,-187.8])
    Coordinate_System.travel_to(15, [100.0,-187.9])
    Coordinate_System.travel_to(15, [95.6,-187.8])
    Coordinate_System.travel_to(40, [10.2,-187.9]) #P22
    Coordinate_System.travel_to(30, [-145.8,-190.9]) #P23 
    Coordinate_System.travel_to(20, [-152.0,-190.7])
    Coordinate_System.travel_to(10, [-158.5,-189.7])
    Coordinate_System.travel_to(15, [-166.2,-187.9])
    Coordinate_System.travel_to(15, [-193.3,-177.6])
    Coordinate_System.travel_to(15, [-196.6,-175.9])
    Coordinate_System.travel_to(15, [-201.3,-172.6])
    Coordinate_System.travel_to(15, [-205.4,-169.2])
    Coordinate_System.travel_to(15, [-212.5,-161.2])
    Coordinate_System.travel_to(15, [-219.7,-150.9])
    Coordinate_System.travel_to(15, [-225.2,-141.4])
    Coordinate_System.travel_to(15, [-229.0,-132.3])
    Coordinate_System.travel_to(15, [-230.7,-125.9])
    Coordinate_System.travel_to(15, [-232.4,-118.1])
    Coordinate_System.travel_to(15, [-233.5,-111.5])
    Coordinate_System.travel_to(15, [-233.8,-103.8])
    Coordinate_System.travel_to(20, [-233.6,-84.3])
    Coordinate_System.travel_to(40, [-232.6,28.1]) #P24 
    Coordinate_System.travel_to(30, [-233.9,98.7])
    Coordinate_System.travel_to(20, [-224.3,132.8])
    Coordinate_System.travel_to(20, [-210.5,151.2])
    Coordinate_System.travel_to(20, [-187.9,176.2])
    Coordinate_System.travel_to(20, [-176.2,182.5])
    Coordinate_System.travel_to(20, [-163.7,186.8])
    Coordinate_System.travel_to(20, [-125.8,186.5])
    Coordinate_System.travel_to(20, [-119.4,186.6]) #P25 

  

                        #[-145.80,-190.90,8.60],#P23
                        #[-232.60,28.10,10.00], #P24
                        #[-119.40,186.60,10.00]
    '''

    while not rospy.is_shutdown():
        rospy.ROSInterruptException  # allow control+C to exit the program        
        Movement_Control.carControl(targetSpeed = 0,steerAngle = 0)
        rate.sleep()
        # rospy.spin()



    

    





#################################################################################################################################################
if __name__ == '__main__':
    try:
        # single time setup
        # test()
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
