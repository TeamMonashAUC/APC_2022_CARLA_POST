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

from shell_simulation.msg import ActualCoord

#################################################################################################################################################
# import libraries
import rospy
import math
import numpy as np

#################################################################################################################################################


def R1():
    Coordinate_System.travel_to2(20, [-206.4, 4.2]) # P1
    Coordinate_System.travel_to2(20, [-245.5,0.8]) 
    Coordinate_System.travel_to2(20, [-255.9,0.2]) # P2   #stop

    Coordinate_System.corner2(20,9,180,[-272.5,-18.7],-90) #turn left
    Coordinate_System.travel_to2(20, [-272.1,-43.9]) # P3
    Coordinate_System.travel_to2(20, [-272.0,-68.2]) 

    # curve
    Coordinate_System.travel_to2(20, [-271.8,-71.9]) 
    Coordinate_System.travel_to2(20, [-271.0,-75.2]) 
    Coordinate_System.travel_to2(20, [-267.8,-81.2]) 
    Coordinate_System.travel_to2(20, [-266.0,-83.1])
    Coordinate_System.travel_to2(20, [-263.5,-85.1]) 
    Coordinate_System.travel_to2(20, [-258.9,-87.5]) 
    Coordinate_System.travel_to2(20, [-252.5,-89.6])
    Coordinate_System.travel_to2(20, [-244.9,-91.0])  
    Coordinate_System.travel_to2(20, [-239.0,-91.3])  


    Coordinate_System.travel_to2(20, [-219.0,-94.6]) 
    Coordinate_System.travel_to2(20, [-205.5,-95.0]) # P4
    Coordinate_System.travel_to2(20, [-205.3,-94.7]) # traffic light 

def R2():

    #########
    Coordinate_System.travel_to2(20, [-195.0,-118.3]) 
    Coordinate_System.travel_to2(20, [-194.4,-123.9]) 
    Coordinate_System.travel_to2(20, [-193.7,-127.4]) 
    Coordinate_System.travel_to2(20, [-193.0,-129.6]) 
    Coordinate_System.travel_to2(20, [-191.7,-132.8]) 
    Coordinate_System.travel_to2(20, [-190.0,-136.1]) 
    Coordinate_System.travel_to2(20, [-187.1,-140.4]) 
    ### P5 pass thru here
    Coordinate_System.travel_to2(20, [-183.1,-144.5]) 
    Coordinate_System.travel_to2(20, [-179.0,-146.9]) 
    Coordinate_System.travel_to2(20, [-171.6,-149.4]) 
    Coordinate_System.travel_to2(20, [-166.8,-150.3]) 
    Coordinate_System.travel_to2(20, [-162.9,-150.7]) 
    # Coordinate_System.travel_to2(15, [-165.0,-153.9]) 

    Coordinate_System.travel_to2(20, [-158.0,-151.1]) 
    Coordinate_System.travel_to2(20, [-151.0,-151.0]) #P6 
    Coordinate_System.travel_to2(20, [-145.6,-151.0]) # traffic light 

def R3():
    # Straight line [From P6 ish to P35]
    Coordinate_System.corner2(20,9,0,[-121.4,-131.4],90)
    # Coordinate_System.travel_to(20, [-123.4,-134.4])aw
    Coordinate_System.travel_to(20, [-123.4,-117.5 ])
    Coordinate_System.travel_to(20, [-123.8,-106.5])
    Coordinate_System.travel_to(20, [-123.9,-74.0])
    Coordinate_System.travel_to(20, [-124.0,-66.6])
    Coordinate_System.travel_to(20, [-124.5,-18.1])
    Coordinate_System.travel_to(20, [-125.0,13.5])
    Coordinate_System.travel_to(20, [-125.0,17.5])
    Coordinate_System.travel_to(20, [-124.7,73.0])
    Coordinate_System.travel_to(20, [-124.4,104.6])
    Coordinate_System.travel_to(20, [-124.4,123.3]) #P35

    Coordinate_System.travel_to(25, [-123.9,135.0])
    Coordinate_System.travel_to(25, [-107.6,138.7])
    Coordinate_System.travel_to(25, [-96.6,138.6])
    Coordinate_System.travel_to(25, [-82.6,134.5])
    Coordinate_System.travel_to(25, [-80.2,133.3]) #P36
    Coordinate_System.travel_to(25, [-75.7,131.8])
    Coordinate_System.travel_to(25, [-75.0,131.4])
    Coordinate_System.travel_to(25, [-71.7,129.6])
    Coordinate_System.travel_to(25, [-66.0,125.1])
    Coordinate_System.travel_to(25, [-60.5,118.5])
    Coordinate_System.travel_to(25, [-58.4,114.9])
    Coordinate_System.travel_to(25, [-55.7,106.1])
    Coordinate_System.travel_to(25, [-55.5,104.7])

def R4():
    # Going to the next line [After P36 going to P10] 
    Coordinate_System.travel_to(20, [-54.8,75.7])
    Coordinate_System.travel_to(20, [-54.8,14.7])
    
    # U turn [at P10]
    Coordinate_System.travel_to(20, [-54.2,-15.3])
    Coordinate_System.travel_to(20, [-52.8,-22.3])
    Coordinate_System.travel_to(20, [-49.9,-24.2])
    Coordinate_System.travel_to(20, [-46.5,-23.4])
    Coordinate_System.travel_to(20, [-44.8,-21.7])
    Coordinate_System.travel_to(20, [-44.4,-20.2])
    Coordinate_System.travel_to(20, [-44.2,-18.7])
    Coordinate_System.travel_to(20, [-43.9,-17.1]) # P10

def R5():
    # Going to the next line [From P11 to P14 loop] 
    Coordinate_System.corner(20,5,90,[-33.9,-2.6],0) #turn right
    Coordinate_System.travel_to(20, [3.0,-2.7]) #P11
    Coordinate_System.travel_to(20, [15.0,-2.6])
    Coordinate_System.travel_to(20, [43.6,-1.8]) 
    Coordinate_System.travel_to(20, [47.8,-1.8]) #P12
    Coordinate_System.travel_to(20, [66.4,-5.2]) 
    Coordinate_System.travel_to(20, [89.0,-5.5]) #P13
    Coordinate_System.travel_to(25, [95.0,-5.5])
    Coordinate_System.travel_to(25, [99.6,-13.4])  
    Coordinate_System.travel_to(25, [99.4,-40.1]) 
    Coordinate_System.travel_to(25, [95.0,-52.0]) 
    Coordinate_System.travel_to(25, [92.8,-59.7]) 
    Coordinate_System.travel_to(25, [88.8,-67.3]) 
    Coordinate_System.travel_to(25, [83.4,-73.5]) 
    Coordinate_System.travel_to(25, [76.3,-79.0]) 
    Coordinate_System.travel_to(25, [65.9,-83.4]) 
    Coordinate_System.travel_to(25, [55.2,-84.9])   
    Coordinate_System.travel_to(25, [45.9,-84.9])  #P14

def R6():
    # Straight line [From P14 ish to P16 ish]
    Coordinate_System.travel_to(25, [35.0,-84.9])
    Coordinate_System.travel_to(25, [31.7,-74.6])
    Coordinate_System.travel_to(25, [31.7,-71.4])
    Coordinate_System.travel_to(25, [31.7,-14.3])
    Coordinate_System.travel_to(25, [31.7,14.9]) 
    Coordinate_System.travel_to(25, [31.3,19.3]) #P15
    Coordinate_System.travel_to(25, [33.1,73.9])
    Coordinate_System.travel_to(25, [37.2,105.6]) 
    Coordinate_System.travel_to(20, [38,132.2])

def R7():
    # Going in into P26 
    Coordinate_System.corner(15,5,90,[47.9,146.0],0) #turn right
    Coordinate_System.travel_to(15, [47.9,146.0]) 
    Coordinate_System.travel_to(15, [84.70,144.10]) #P26
    Coordinate_System.travel_to(15, [95.9,146.4])
    Coordinate_System.travel_to(15, [93.9,146.7])
    Coordinate_System.travel_to(15, [74.8,148.7])
    Coordinate_System.travel_to(15, [48.0,150.1])
    Coordinate_System.travel_to(15, [42.2,150.3])
    Coordinate_System.travel_to(15, [35.3,164.3])
    Coordinate_System.travel_to(25, [35.9,182.2])
    
def R14():#Final ring

    # curve
    Coordinate_System.travel_to2(20, [57.6,190.3])
    Coordinate_System.travel_to2(20, [74.0,190.5])   #P18 (74.0, 190.0)

    Coordinate_System.travel_to2(20, [100.9,190.4])
    Coordinate_System.travel_to2(20, [112.6,189.6])
    Coordinate_System.travel_to2(20, [124.9,187.0])
    Coordinate_System.travel_to2(20, [137.6,182.7])
    Coordinate_System.travel_to2(20, [146.1,180.2])
    Coordinate_System.travel_to2(20, [151.8,178.3])
    # Coordinate_System.travel_to2(0, [153.0,178.4])
    # Coordinate_System.travel_to2(20, [154.10,177.20])  #P19

    Coordinate_System.travel_to2(20, [154.10,176.20])  #P19

    # Coordinate_System.travel_to2(20, [161.3,171.6]) 
    Coordinate_System.travel_to2(20, [171.8,157.2])  
    Coordinate_System.travel_to2(20, [182.8,132.3])  
    Coordinate_System.travel_to2(20, [187.3,113.9])   

    # straight
    Coordinate_System.travel_to2(20, [188.6,97.4])    
    Coordinate_System.travel_to2(20, [189.2,51.9])     
    Coordinate_System.travel_to2(20, [189.8,-31.6])   

    # curve
    Coordinate_System.travel_to2(20, [188.9,-98.8]) 
    Coordinate_System.travel_to2(20, [186.6,-119.9]) 
    Coordinate_System.travel_to2(20, [180.6,-137.3]) 
    Coordinate_System.travel_to2(20, [168.7,-155.7]) 
    Coordinate_System.travel_to2(20, [153.8,-170.1]) 
    Coordinate_System.travel_to2(20, [137.1,-180.0]) 
    Coordinate_System.travel_to2(20, [118.5,-186.1]) 
    Coordinate_System.travel_to2(20, [118.5,-186.1])     

    # straight
    Coordinate_System.travel_to2(20, [98.8,-188.0])    
    Coordinate_System.travel_to2(20, [46.6,-187.8])     
    Coordinate_System.travel_to2(20, [10.2,-187.9]) #P22

    Coordinate_System.travel_to2(20, [-76.7,-187.5])    
    Coordinate_System.travel_to2(20, [-144.0,-187.5])      

    # curve
    Coordinate_System.travel_to2(20, [-155.2,-186.4]) 
    Coordinate_System.travel_to2(20, [-164.2,-184.6]) 
    Coordinate_System.travel_to2(20, [-169.0,-183.0]) 
    Coordinate_System.travel_to2(20, [-171.4,-182.1]) 
    Coordinate_System.travel_to2(20, [-176.2,-180.2]) 
    Coordinate_System.travel_to2(20, [-180.9,-178.0]) 
    Coordinate_System.travel_to2(20, [-189.9,-172.4]) 
    Coordinate_System.travel_to2(20, [-198.4,-165.7]) 
    Coordinate_System.travel_to2(20, [-205.7,-158.5]) 
    Coordinate_System.travel_to2(20, [-216.1,-144.1]) 
    Coordinate_System.travel_to2(20, [-222.3,-129.7]) 
    Coordinate_System.travel_to2(20, [-226.1,-113.3]) 

    # straight
    Coordinate_System.travel_to2(18, [-232.5,-75.6]) 
    Coordinate_System.travel_to2(13, [-232.6,-29.3]) 

    Coordinate_System.travel_to2(13, [-232.6,28.1]) #P24 




    # Coordinate_System.travel_to(25, [50.0,190.2])
    # Coordinate_System.travel_to(20, [52.0,190.5])
    # Coordinate_System.travel_to(20, [54.0,190.5]) 
    # Coordinate_System.travel_to(20, [74.0,190.5])   #P18 (74.0, 190.0)

    # # Cutting to the side lane 
    # Coordinate_System.travel_to(20, [95.1,193.7])
    # Coordinate_System.travel_to(20, [95.5,193.7])
    # Coordinate_System.travel_to(20, [108.0,193.7])
    # Coordinate_System.travel_to(20, [115.8,192.5])
    # Coordinate_System.travel_to(20, [122.0,191.5])
    # Coordinate_System.travel_to(20, [126.0,190.5])
    # Coordinate_System.travel_to(20, [131.5,188.8])
    # Coordinate_System.travel_to(20, [135.0,187.5])
    # Coordinate_System.travel_to(20, [140.0,185.5])
    # Coordinate_System.travel_to(20, [145.7,182.2])
    # Coordinate_System.travel_to(20, [148.1,181.0])  
    # Coordinate_System.travel_to(20, [154.1,177.3])  #P19
    # Coordinate_System.travel_to(20, [165.0,168.0])
    # Coordinate_System.travel_to(20, [174.5,158.0])
    # Coordinate_System.travel_to(20, [180.2,150.6])
    # Coordinate_System.travel_to(20, [184.1,143.5])
    # Coordinate_System.travel_to(20, [187.5,137.1])
    # Coordinate_System.travel_to(20, [190.0,120.7])
    # Coordinate_System.travel_to(20, [192.5,121.4])
    # Coordinate_System.travel_to(20, [193.7,117.3])
    # Coordinate_System.travel_to(20, [195.6,101.3])
    # Coordinate_System.travel_to(20, [189.1,72.0])   #189.3?
    # Coordinate_System.travel_to(20, [189.2,52.8])   #P20
    # Coordinate_System.travel_to(20, [189.2,50.0])
    # Coordinate_System.travel_to(20, [190.0,-4.0])
    # Coordinate_System.travel_to(20, [189.5,-80.0])
    # Coordinate_System.travel_to(20, [188.9,-104.7])
    # Coordinate_System.travel_to(20, [187.4,-116.4])
    # Coordinate_System.travel_to(20, [183.9,-129.5])
    # Coordinate_System.travel_to(20, [181.5,-135.7])
    # Coordinate_System.travel_to(20, [179.0,-140.0])
    # Coordinate_System.travel_to(20, [175.5,-146.0])
    # Coordinate_System.travel_to(20, [174.4,-148.0]) #P21
    # Coordinate_System.travel_to(20, [168.5,-156.0])
    # Coordinate_System.travel_to(20, [156.6,-168.0])
    # Coordinate_System.travel_to(20, [143.5,-176.9])
    # Coordinate_System.travel_to(20, [134.7,-180.9])
    # Coordinate_System.travel_to(20, [123.1,-185.0])
    # Coordinate_System.travel_to(20, [111.1,-187.4])
    # Coordinate_System.travel_to(20, [104.2,-187.8])
    # Coordinate_System.travel_to(40, [100.0,-187.9])
    # Coordinate_System.travel_to(40, [95.6,-187.8])
    # Coordinate_System.travel_to(40, [10.2,-187.9]) #P22
    # Coordinate_System.travel_to(40, [-145.8,-190.9]) #P23 
    # Coordinate_System.travel_to(40, [-152.0,-190.7])
    # Coordinate_System.travel_to(20, [-158.5,-189.7])
    # Coordinate_System.travel_to(20, [-166.2,-187.9])
    # Coordinate_System.travel_to(20, [-193.3,-177.6])
    # Coordinate_System.travel_to(20, [-196.6,-175.9])
    # Coordinate_System.travel_to(20, [-201.3,-172.6])
    # Coordinate_System.travel_to(20, [-205.4,-169.2])
    # Coordinate_System.travel_to(20, [-212.5,-161.2])
    # Coordinate_System.travel_to(20, [-219.7,-150.9])
    # Coordinate_System.travel_to(20, [-225.2,-141.4])    
    # Coordinate_System.travel_to(20, [-229.0,-132.3])
    # Coordinate_System.travel_to(20, [-220.7,-125.9])
    # Coordinate_System.travel_to(20, [-232.4,-118.1])
    # Coordinate_System.travel_to(20, [-233.5,-111.5])
    # Coordinate_System.travel_to(20, [-233.8,-103.8])
    # Coordinate_System.travel_to(20, [-233.6,-84.3])
    # Coordinate_System.travel_to(40, [-232.6,28.1]) #P24

def main():
    # Coordinate_System.update_Coord()
    # '''
    rospy.loginfo(settings.car_coordinate_from_world)
    while settings.car_coordinate_from_world[0] ==0:      
        Movement_Control.carControl(targetSpeed = 0,steerAngle = 0)
        rospy.loginfo(settings.car_coordinate_from_world)
        pass

    rospy.loginfo("start coord:")
    rospy.loginfo(settings.car_coordinate_from_world)
    # Coordinate_System.travel_to(20, [-171.60,4.00,0.00]) #P0
    R1()
    Coordinate_System.corner2(20,6,0,[-195.1,-105.1],-90) #turn right
    R2()
    R3()
    R4()
    R5()
    R6()
    R7()
    R14()
    
    while not rospy.is_shutdown():
        rospy.ROSInterruptException  # allow control+C to exit the program        
        Movement_Control.carControl(targetSpeed = 0,steerAngle = 0)
        rate.sleep()
        # rospy.spin()
    # '''


    

    





#################################################################################################################################################
if __name__ == '__main__':
    # try:


    # single time setup
    # test()
    # start rosnode
    rospy.init_node('APC_Monash2')
    rate = rospy.Rate(100) # publish data at 100Hz
    rospy.loginfo("APC_Monash started")

    # start ros communications with rostopics
    ROS_Communication.ROS_Start()


    # global pub_coord_2D
    # global pub_coord_3D 
    # pub_coord_3D = rospy.Publisher("/actual_coord_3D", ActualCoord, queue_size = 10)

    while not rospy.is_shutdown():
    #try: 
        # infinite loop
        main()
        rate.sleep()
 
    # except ValueError:
    #     print("Oops!  That was no valid number.  Try again...")

    # except rospy.ROSInterruptException: # if we stop the script (using CTRL+C), it will run rospy.ROSInterruptException
        
    #     rospy.loginfo("Exit program successful") # Exit message
    #     pass

'''
def R3():
    Coordinate_System.travel_to(25, [-83.20,-87.70]) #P32
    Coordinate_System.travel_to(20, [-105.9,-87.8]) 
    
    Coordinate_System.corner(15,5,180,[-124.1,-72.3],90) #turn right
    Coordinate_System.travel_to(20, [-124.20,-42.40]) # P33
    Coordinate_System.travel_to(20, [-124.50,-17.5])
    Coordinate_System.travel_to(40, [-124.50,73.0])  
    
def R4():
    Coordinate_System.travel_to(40, [-124.4,106.3])
    Coordinate_System.travel_to(20, [-124.4,123.3]) #P35
    Coordinate_System.travel_to(25, [-123.9,136.8])
    Coordinate_System.travel_to(25, [-107.6,138.7])
    Coordinate_System.travel_to(25, [-96.6,138.6])
    Coordinate_System.travel_to(25, [-82.6,134.5])
    Coordinate_System.travel_to(25, [-80.2,133.3]) #P36
    Coordinate_System.travel_to(25, [-75.7,131.8])
    Coordinate_System.travel_to(25, [-75.0,131.4])
    Coordinate_System.travel_to(25, [-71.7,129.6])
    Coordinate_System.travel_to(25, [-66.0,125.1])
    Coordinate_System.travel_to(25, [-60.5,118.5])
    Coordinate_System.travel_to(25, [-58.4,114.9])
    Coordinate_System.travel_to(25, [-55.7,106.1])
    Coordinate_System.travel_to(25, [-55.5,104.7])
    Coordinate_System.travel_to(25, [-54.8,97.8])
    Coordinate_System.travel_to(25, [-35.6,87.8])

def R5():
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
    Coordinate_System.travel_to(20, [116.5,2.2]) 

def R7():
    Coordinate_System.travel_to(25, [96.2,-14.9])
    Coordinate_System.travel_to(25, [96.20,-28.60]) #P20

    #curve
    Coordinate_System.travel_to(25, [95.70,-43.60]) 
    Coordinate_System.travel_to(25, [95,-52]) 
    Coordinate_System.travel_to(25, [92.8,-59.7]) 
    Coordinate_System.travel_to(25, [88.8,-67.3]) 
    Coordinate_System.travel_to(25, [83.4,-73.5]) 
    Coordinate_System.travel_to(25, [76.3,-79]) 
    Coordinate_System.travel_to(25, [65.9,-83.4]) 
    Coordinate_System.travel_to(25, [55.2,-84.9]) 

    Coordinate_System.travel_to(15, [45.90,-84.90]) #P20
    Coordinate_System.travel_to(15, [45.20,-84.90]) 

def R8(): 
    Coordinate_System.travel_to(20, [-32.8,-88.0]) #P31

def R9():
    Coordinate_System.travel_to(20, [-43.7,-73.9]) 
    Coordinate_System.travel_to(25, [-43.80,-56.80]) #P9
    Coordinate_System.travel_to(20, [-43.5,-16.9]) #P10

    Coordinate_System.corner(20,5,90,[-33.9,-2.6],0) #turn right
    Coordinate_System.travel_to(20, [3.00,-2.70]) #P11
    Coordinate_System.travel_to(20, [14.40,-2.60]) 
    

def R10():
    Coordinate_System.travel_to(25, [24.8,-19.3]) 
    Coordinate_System.travel_to(25, [24.60,-20.70]) #P39
    Coordinate_System.travel_to(25, [24.4,-72.3]) 


def R11():
    Coordinate_System.travel_to(15, [44.1,-1.9]) 
    Coordinate_System.travel_to(15, [47.80,-1.80]) #P12
    Coordinate_System.travel_to(15, [66.4,-5.2]) 
    Coordinate_System.travel_to(15, [89.00,-5.50]) #P13

def R12():
    Coordinate_System.travel_to(25, [25.70,65.40]) #P38
    Coordinate_System.travel_to(25, [27.8,15.8]) 

def R13():

    Coordinate_System.travel_to(20, [31.6,15.4]) 
    Coordinate_System.travel_to(20, [31.20,19.20]) #P15

    Coordinate_System.travel_to(20, [35.4,46.2]) 
    Coordinate_System.travel_to(25, [36.20,67.2]) #P16
    Coordinate_System.travel_to(25, [36.5,74.5]) 
'''
