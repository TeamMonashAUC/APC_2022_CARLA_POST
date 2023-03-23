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
    # Coordinate_System.travel_to2(20, [-245.5,0.8]) 
    # Coordinate_System.travel_to2(20, [-255.9,0.2]) # P2   #stop

    Coordinate_System.corner2(20,15,180,[-272.5,-43.9],-90) #turn left
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


    # Coordinate_System.travel_to2(20, [-219.0,-94.6]) 
    # Coordinate_System.travel_to2(20, [-205.5,-95.0]) # P4
    # Coordinate_System.travel_to2(20, [-205.3,-94.7]) # traffic light 
    # Coordinate_System.travel_to2(25, [-205.3,-91.4]) # traffic light 

def R2():
    # curve
    # Coordinate_System.travel_to2(20, [-199.1,-95.0])
    # Coordinate_System.travel_to2(25, [-195.0,-104.6]) 
    Coordinate_System.travel_to2(20, [-195.0,-118.3]) 
    Coordinate_System.travel_to2(20, [-193.9,-124.9]) 
    Coordinate_System.travel_to2(20, [-188.8,-134.2]) 
    Coordinate_System.travel_to2(20, [-183.6,-140.5]) 
    Coordinate_System.travel_to2(20, [-176.2,-146.1]) 
    Coordinate_System.travel_to2(20, [-167.5,-149.7]) 
    Coordinate_System.travel_to2(20, [-162.6,-150.6]) 
    # Coordinate_System.travel_to2(20, [-183.1,-144.5]) 
    # Coordinate_System.travel_to2(20, [-179.0,-146.9]) 
    # Coordinate_System.travel_to2(20, [-171.6,-149.4]) 
    # Coordinate_System.travel_to2(20, [-166.8,-150.3]) 
    # Coordinate_System.travel_to2(20, [-162.9,-150.7]) 
    # Coordinate_System.travel_to2(15, [-165.0,-153.9]) 

    Coordinate_System.travel_to2(20, [-158.0,-151.1]) 
    Coordinate_System.travel_to2(20, [-151.0,-151.0]) #P6 
    Coordinate_System.travel_to2(20, [-145.6,-151.0]) # traffic light 
    # Coordinate_System.travel_to2(20, [-191.6,-116.6])
    # Coordinate_System.travel_to2(20, [-191.3,-121.5])
    # Coordinate_System.travel_to2(20, [-190.2,-126.5])
    # Coordinate_System.travel_to2(20, [-188.4,-131.5])
    # Coordinate_System.travel_to2(20, [-187.1,-134.3])
    # Coordinate_System.travel_to2(20, [-185.0,-137.3])
    # Coordinate_System.travel_to2(20, [-182.6,-140.2])
    # Coordinate_System.travel_to2(20, [-178.3,-144.0])
    # Coordinate_System.travel_to2(20, [-174.2,-146.8])
    # Coordinate_System.travel_to2(20, [-169.0,-149.1])
    # Coordinate_System.travel_to2(20, [-162.6,-150.7])
    # Coordinate_System.travel_to2(20, [-155.4,-151.0])
    # Coordinate_System.travel_to2(20, [-145.9,-151.2])

    #crossing junction
    Coordinate_System.travel_to2(20, [-111.1,-151.2])

    #curve
    Coordinate_System.travel_to2(20, [-86.5,-151.2])
    Coordinate_System.travel_to2(20, [-80.7,-150.5])
    Coordinate_System.travel_to2(20, [-74.5,-148.8])
    Coordinate_System.travel_to2(20, [-69.2,-146.4])
    Coordinate_System.travel_to2(20, [-63.4,-142.7])
    Coordinate_System.travel_to2(20, [-58.8,-138.5])
    Coordinate_System.travel_to2(20, [-54.3,-133.1])
    Coordinate_System.travel_to2(20, [-51.6,-128.6])
    Coordinate_System.travel_to2(20, [-49.4,-122.9])
    Coordinate_System.travel_to2(20, [-48.2,-117.5])
    Coordinate_System.travel_to2(20, [-47.6,-112.2])
    Coordinate_System.travel_to2(20, [-47.5,-105.5])

    pass
    # curve
    # Coordinate_System.travel_to2(20, [-199.1,-95.0])
    # Coordinate_System.travel_to2(25, [-195.0,-104.6]) 
    # Coordinate_System.travel_to2(25, [-195.0,-118.3]) 
    # Coordinate_System.travel_to2(25, [-193.7,-127.2]) 
    # Coordinate_System.travel_to2(25, [-191.8,-132.4]) 
    # Coordinate_System.travel_to2(25, [-183.9,-144.2]) 
    # Coordinate_System.travel_to2(25, [-175.5,-150.1]) 
    # # Coordinate_System.travel_to2(15, [-165.0,-153.9]) 

    # Coordinate_System.travel_to2(25, [-158.0,-151.1]) 
    # Coordinate_System.travel_to2(25, [-151.0,-151.0]) #P6 
    # Coordinate_System.travel_to2(35, [-145.6,-151.0]) # traffic light 


    # Coordinate_System.travel_to2(35, [-121.0,-151.1]) 
    # Coordinate_System.travel_to2(25, [-101.4,-154.7]) # P7

    # #curve
    # Coordinate_System.travel_to2(25, [-85.8,-154.9]) 
    # Coordinate_System.travel_to2(25, [-75.0,-152.7]) 
    # Coordinate_System.travel_to2(25, [-65.8,-148.7]) 
    # Coordinate_System.travel_to2(25, [-54.4,-136.6]) 

    # Coordinate_System.travel_to2(25, [-47.8,-117.2]) # P8 
    # Coordinate_System.travel_to2(25, [-47.4,-105.9]) 

def R3():
    Coordinate_System.travel_to2(20, [-47.2,-73.2])
    Coordinate_System.travel_to2(20, [-47.2,-68.2])

    #swap lanes
    Coordinate_System.travel_to2(20, [-45.6,-52.3])
    Coordinate_System.travel_to2(20, [-43.9,-36.7])
    Coordinate_System.travel_to2(20, [-43.9,-25.0])
    Coordinate_System.travel_to2(20, [-43.90,-17.10,0.00]) #P10

    Coordinate_System.corner2(20,10,90,[-24.2,-3.1],0) #turn right
    # Coordinate_System.travel_to2(20, [-18.6,-2.70])
    Coordinate_System.travel_to2(20, [3.00,-2.70]) #P11
    # Coordinate_System.travel_to2(20, [13.7,-2.60])

'''
    Coordinate_System.corner2(20,10,0,[32.0,19.3],90) #turn left
    Coordinate_System.travel_to2(20, [31.30,19.30]) # P15 

    # Coordinate_System.travel_to2(20, [34.6,51.60]) 
    # Coordinate_System.travel_to2(20, [36.4,72.20])

    # # cross x junction
    # Coordinate_System.travel_to2(20, [37.2,105.0])
    # Coordinate_System.travel_to2(20, [38.1,133.2])
    Coordinate_System.travel_to2(20, [34.3,122.9])
     

    # Coordinate_System.travel_to2(25, [-83.20,-87.70]) #P32
    # Coordinate_System.travel_to2(20, [-105.9,-87.8]) 
    
    # Coordinate_System.corner2(15,5,180,[-124.1,-72.3],90) #turn right
    # Coordinate_System.travel_to2(20, [-124.30,-42.40]) # P33
    # Coordinate_System.travel_to2(30, [-124.50,-17.5])
    # Coordinate_System.travel_to2(40, [-124.50,73.0])  
''' 

def R6():
    # Coordinate_System.travel_to2(15, [47.9,146.0]) 
    Coordinate_System.travel_to2(20, [84.70,144.10]) #P26

    #curve
    Coordinate_System.travel_to2(20, [94,142.7]) 
    Coordinate_System.travel_to2(20, [103.8,141.1]) 
    Coordinate_System.travel_to2(20, [114.3,138.8]) 
    Coordinate_System.travel_to2(20, [124.4,134.7]) 
    Coordinate_System.travel_to2(20, [132.8,129.8]) 
    Coordinate_System.travel_to2(20, [138.6,125.0]) 
    Coordinate_System.travel_to2(20, [143.1,119.7]) 
    Coordinate_System.travel_to2(20, [146.1,115.1]) 
    Coordinate_System.travel_to2(20, [148.10,112.20]) #P27 
    Coordinate_System.travel_to2(20, [150.0,106.60]) 
    Coordinate_System.travel_to2(20, [150.8,101.40]) 
    Coordinate_System.travel_to2(20, [151.3,94.4]) 
    Coordinate_System.travel_to2(20, [151.4,87.1]) 

    Coordinate_System.travel_to2(20, [151.4,13.2]) 
    Coordinate_System.corner2(20,6,-90,[141.4,2.2],-180) #turn right
    Coordinate_System.travel_to2(20, [124.70,1.9]) #P29
    # Coordinate_System.travel_to2(20, [116.5,2.2]) 

def R7():
    # Coordinate_System.travel_to2(20, [96.2,-14.9])
    Coordinate_System.travel_to2(20, [96.20,-28.60]) #P30

    #curve
    Coordinate_System.travel_to2(20, [95.70,-43.60]) 
    Coordinate_System.travel_to2(20, [95,-52]) 
    Coordinate_System.travel_to2(20, [92.8,-59.7]) 
    Coordinate_System.travel_to2(20, [88.8,-67.3]) 
    Coordinate_System.travel_to2(20, [83.4,-73.5]) 
    Coordinate_System.travel_to2(20, [76.3,-79]) 
    Coordinate_System.travel_to2(20, [65.9,-83.4]) 
    Coordinate_System.travel_to2(20, [55.2,-84.9]) 

    Coordinate_System.travel_to2(20, [45.90,-84.90]) #P14
    Coordinate_System.travel_to2(20, [45.30,-84.90]) 
    print("finish R7 route")

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
    # Coordinate_System.travel_to2(20, [154.10,177.30])  #P19

    Coordinate_System.travel_to2(20, [154.10,176.30])  #P19

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
    Coordinate_System.travel_to2(18, [-232.6,-29.3]) 

    Coordinate_System.travel_to2(15, [-232.6,28.1]) #P24 
    '''
    Coordinate_System.travel_to2(25, [50.0,190.2])
    Coordinate_System.travel_to2(20, [52.0,190.5])
    Coordinate_System.travel_to2(20, [54.0,190.5]) 
    Coordinate_System.travel_to2(20, [74.0,190.5])   #P18 (74.0, 190.0)

    # Cutting to the side lane 
    Coordinate_System.travel_to2(20, [95.1,193.7])
    Coordinate_System.travel_to2(20, [95.5,193.7])
    Coordinate_System.travel_to2(20, [108.0,193.7])
    Coordinate_System.travel_to2(20, [115.8,192.5])
    Coordinate_System.travel_to2(20, [122.0,191.5])
    Coordinate_System.travel_to2(20, [126.0,190.5])
    Coordinate_System.travel_to2(20, [131.5,188.8])
    Coordinate_System.travel_to2(20, [135.0,187.5])
    Coordinate_System.travel_to2(20, [140.0,185.5])
    Coordinate_System.travel_to2(20, [145.7,182.2])
    Coordinate_System.travel_to2(20, [148.1,181.0])  
    Coordinate_System.travel_to2(20, [154.1,177.3])  #P19
    Coordinate_System.travel_to2(20, [165.0,168.0])
    Coordinate_System.travel_to2(20, [174.5,158.0])
    Coordinate_System.travel_to2(20, [180.2,150.6])
    Coordinate_System.travel_to2(20, [184.1,143.5])
    Coordinate_System.travel_to2(20, [187.5,137.1])
    Coordinate_System.travel_to2(20, [190.0,130.7])
    Coordinate_System.travel_to2(20, [192.5,121.4])
    Coordinate_System.travel_to2(20, [193.7,117.3])
    Coordinate_System.travel_to2(20, [195.6,101.3])
    Coordinate_System.travel_to2(20, [189.1,72.0])   #189.3?
    Coordinate_System.travel_to2(20, [189.2,52.8])   #P20
    Coordinate_System.travel_to2(30, [189.2,50.0])
    Coordinate_System.travel_to2(30, [190.0,-4.0])
    Coordinate_System.travel_to2(30, [189.5,-80.0])
    Coordinate_System.travel_to2(30, [188.9,-104.7])
    Coordinate_System.travel_to2(30, [187.4,-116.4])
    Coordinate_System.travel_to2(30, [183.9,-129.5])
    Coordinate_System.travel_to2(30, [181.5,-135.7])
    Coordinate_System.travel_to2(30, [179.0,-140.0])
    Coordinate_System.travel_to2(30, [175.5,-146.0])
    Coordinate_System.travel_to2(30, [174.4,-148.0]) #P21
    Coordinate_System.travel_to2(30, [168.5,-156.0])
    Coordinate_System.travel_to2(30, [156.6,-168.0])
    Coordinate_System.travel_to2(30, [143.5,-176.9])
    Coordinate_System.travel_to2(30, [134.7,-180.9])
    Coordinate_System.travel_to2(30, [123.1,-185.0])
    Coordinate_System.travel_to2(30, [111.1,-187.4])
    Coordinate_System.travel_to2(30, [104.2,-187.8])
    Coordinate_System.travel_to2(40, [100.0,-187.9])
    Coordinate_System.travel_to2(40, [95.6,-187.8])
    Coordinate_System.travel_to2(40, [10.2,-187.9]) #P22
    Coordinate_System.travel_to2(40, [-145.8,-190.9]) #P23 
    Coordinate_System.travel_to2(40, [-152.0,-190.7])
    Coordinate_System.travel_to2(30, [-158.5,-189.7])
    Coordinate_System.travel_to2(30, [-166.2,-187.9])
    Coordinate_System.travel_to2(30, [-193.3,-177.6])
    Coordinate_System.travel_to2(30, [-196.6,-175.9])
    Coordinate_System.travel_to2(30, [-201.3,-172.6])
    Coordinate_System.travel_to2(30, [-205.4,-169.2])
    Coordinate_System.travel_to2(30, [-212.5,-161.2])
    Coordinate_System.travel_to2(30, [50.2,190.0][-219.7,-150.9])
    Coordinate_System.travel_to2(30, [-225.2,-141.4])
    Coordinate_System.travel_to2(30, [-229.0,-132.3])
    Coordinate_System.travel_to2(30, [-230.7,-125.9])
    Coordinate_System.travel_to2(30, [-232.4,-118.1])
    Coordinate_System.travel_to2(30, [-233.5,-111.5])
    Coordinate_System.travel_to2(30, [-233.8,-103.8])
    Coordinate_System.travel_to2(30, [-233.6,-84.3])
    Coordinate_System.travel_to2(40, [-232.6,28.1]) #P24 
    # Coordinate_System.travel_to2(35, [-233.9,98.7])
    # Coordinate_System.travel_to2(30, [-224.3,132.8])
    # Coordinate_System.travel_to2(30, [-210.5,151.2])
    # Coordinate_System.travel_to2(30, [-187.9,176.2])
    # Coordinate_System.travel_to2(30, [-176.2,182.5])
    # Coordinate_System.travel_to2(30, [-163.7,186.8])
    # Coordinate_System.travel_to2(30, [-125.8,186.5])
    # Coordinate_System.travel_to2(30, [-119.4,186.6]) #P25 
    '''

def R15():
    
    Coordinate_System.corner2(16,12,-180,[-187.9,31.1],90) #turn right
    Coordinate_System.travel_to2(20, [-187.9,65.8])  
    Coordinate_System.corner2(20,12,90,[-168.1,84.7],0) #turn right
    Coordinate_System.travel_to2(20, [-156.4,84.7])  
    Coordinate_System.corner2(20,15,0,[-124.40,123.80],90) #turn left
    # Coordinate_System.travel_to2(20, [-124.4,123.8])  
    # Coordinate_System.travel_to2(20, [-124.40,106.30])  #P35
    Coordinate_System.corner2(20,15,90,[-154.80,142.60],180) #turn left
    Coordinate_System.travel_to2(20, [-161.1,142.6])  
    Coordinate_System.travel_to2(20, [-168.5,141.5])  
    Coordinate_System.travel_to2(20, [-175.5,138.6])  
    Coordinate_System.travel_to2(20, [-179.8,135.8])  
    Coordinate_System.travel_to2(20, [-183.7,131.9])  
    Coordinate_System.travel_to2(20, [-187.2,127.3])  
    Coordinate_System.travel_to2(20, [-189.3,123.0])  
    Coordinate_System.travel_to2(20, [-191.4,115.3])  
    Coordinate_System.travel_to2(20, [-191.7,104.8])  

    # cross x junction
    Coordinate_System.travel_to2(20, [-191.6,74.1])  
    # Coordinate_System.travel_to2(20, [-191.5,15.2])  
    Coordinate_System.corner2(20,10,-90,[-206.4, 4.2],180) #turn right

def R16():
    Coordinate_System.travel_to2(20, [43.0, -1.8])
    Coordinate_System.travel_to2(20, [47.0, -1.8]) # P12
    Coordinate_System.travel_to2(20, [87.6, -2.0])
    Coordinate_System.travel_to2(20, [91.8, -2.2])

    # Turning left
    # Might need andrew's corner
    Coordinate_System.travel_to2(20, [103.4, 9.3])
    Coordinate_System.travel_to2(20, [103.4, 12.9])
    Coordinate_System.travel_to2(20, [103.4, 45.3])
    Coordinate_System.travel_to2(20, [103.2, 50.7])
    Coordinate_System.travel_to2(20, [102.1, 53.6])
    Coordinate_System.travel_to2(20, [98.8, 63.3])
    Coordinate_System.travel_to2(20, [95.7, 71.1])
    Coordinate_System.travel_to2(20, [90.9, 77.1])
    Coordinate_System.travel_to2(20, [85.3, 82.0])
    Coordinate_System.travel_to2(20, [77.2, 86.9])
    Coordinate_System.travel_to2(20, [69.1, 89.9])
    Coordinate_System.travel_to2(20, [66.9, 90.5])
    Coordinate_System.travel_to2(20, [61.1, 91.3])
    Coordinate_System.travel_to2(20, [45.4, 91.2])

    # CORNERING
    Coordinate_System.travel_to2(20, [37.1, 104.3])
















def main():
    # Spawn point {"x": -171.60, "y": 4.0, "z": 0.2, "roll": 0.0, "pitch": 0.0, "yaw": 180.0}

    # section 1 (yellow then green path)
    '''
    R15()
    R1()
    Coordinate_System.corner2(20,12,0,[-195.0,-118.3],-90) #turn right
    R2()

    # Coordinate_System.corner2(20,6,90,[-65.8,-87.9],0) #turn left
    R3()
    '''
    # Steve adds
    R16()

    # section 2 (blue path)
    Coordinate_System.corner2(20,8,90,[49.6,146.1],0) #turn right
    R6()
    # '''
    Coordinate_System.corner2(20,12,-180,[96.20,-28.60],-90) #turn left
    R7()
    Coordinate_System.corner2(20,6,180,[32.3,-60.7],90) #turn right
    Coordinate_System.travel_to2(20, [31.8,-47.4])  
    # Coordinate_System.travel_to2(20, [31.8,-16.1])  


    Coordinate_System.corner2(20,8,90,[43.6,-2.8],0) #turn right
    Coordinate_System.travel_to2(20, [60.3,-3.1]) 
    Coordinate_System.travel_to2(20, [68.6,-4.3]) 
    # Coordinate_System.travel_to2(20, [77.4,-5.4]) 
    # Coordinate_System.travel_to2(20, [88.0,-5.6])  

    Coordinate_System.corner2(20,15,0,[106.9,17.6],90) #turn left
    # Coordinate_System.travel_to2(20, [103.5,34.0])  
    Coordinate_System.travel_to2(20, [103.4,49.0])

    ## here  
    Coordinate_System.travel_to2(20, [102.3,55.7])  
    Coordinate_System.travel_to2(20, [100.4,61.9])  
    Coordinate_System.travel_to2(20, [97.5,68.1])  
    Coordinate_System.travel_to2(20, [92.5,75.1])  
    Coordinate_System.travel_to2(20, [86.3,81.2])   
    Coordinate_System.travel_to2(20, [79.4,85.8])  
    Coordinate_System.travel_to2(20, [72.1,89.0]) 
    Coordinate_System.travel_to2(20, [64.0,90.9]) 
    Coordinate_System.travel_to2(20, [54.2,91.5]) 
    # Coordinate_System.travel_to2(20, [47.2,94.6]) 

    Coordinate_System.corner2(20,12,180,[37.0,119.7],90) #turn right
    Coordinate_System.travel_to2(20, [38.8,155.1]) 
    
    # '''
    # Coordinate_System.travel_to2(20, [39.1,180.2])
    Coordinate_System.corner2(20,12,90,[57.6,190.3],0)
    R14()

    # [-232.6,28.1]

    # # cross x junction
    # Coordinate_System.travel_to2(20, [35.0,14.9])  
    # Coordinate_System.travel_to2(20, [36.4,72.6]) 

    # # cross x junction
    # Coordinate_System.travel_to2(20, [37.2,104.3])  
    # Coordinate_System.travel_to2(20, [38.8,161.2]) 


    # R4()
    # R5()
    # Coordinate_System.corner2(20,5,0,[29.4,75.5],-90) #turn right
    # R12()

    # R10()
    # Coordinate_System.corner2(15,5,-90,[14.7,-88],-180) #turn right
    # R8()
    # Coordinate_System.corner2(15,5,-180,[-43.7,-73.9],90) #turn right
    # R9()
    # R11()
    # Coordinate_System.corner2(15,5,0,[96.2,-14.9],-90) #turn right
    # R7()
    # Coordinate_System.corner2(15,5,-180,[31.7,-68.7],90) #turn right
    # Coordinate_System.travel_to2(15, [31.7,-16.2]) 
    # R13()

    # Coordinate_System.travel_to2(25, [37.2,105.6]) 
    # Coordinate_System.travel_to2(20, [38,132.2]) 
    # Coordinate_System.corner2(15,5,90,[47.9,146.0],0) #turn right
                                                                                                                                                                                                                                                                      
    # R6()

    # Coordinate_System.travel_to2(25, [87.6,5.1]) 
    # Coordinate_System.travel_to2(20, [44.7,5.2]) 
    # Coordinate_System.corner2(15,5,180,[31.6,15.4],90) #turn right

    # R13()
    # Coordinate_System.travel_to2(25, [37.2,105.6]) 
    # Coordinate_System.travel_to2(25, [38,132.2]) 
    # Coordinate_System.travel_to2(25, [39.1,172.8]) 

    # Coordinate_System.travel_to2(15, [39.1,180.2])
    # Coordinate_System.corner2(15,6,90,[48.3,190.2],0)
    # R14()
    

    while not rospy.is_shutdown():
        rospy.ROSInterruptException  # allow control+C to exit the program        
        Movement_Control.carControl(targetSpeed = 0,steerAngle = 0)
        rate.sleep()
# test run andrew original branch:
# test 1: energy_kWh:  0.2934999121632436
# test 2: energy_kWh:  0.17770695611582027
# test 3: energy_kWh:  0.23565287282995398
# test 4: energy_kWh:  0.2513430148776846
# test 5: energy_kWh:  0.21931197243393427
# test 6: energy_kWh:  0.1993555743346252
# average: 0.2295

    

# test run ze xin branch: 
# test 1: energy_kWh:  0.27732951271690326
# test 2: energy_kWh:  0.32768018887852685
# test 3: energy_kWh:  0.22579428262208714
# test 4: energy_kWh:  0.15131076884522943
# test 5: energy_kWh:  0.20301202743055716
# test 6: energy_kWh:  0.24772876110846637
# average = 0.239

#################################################################################################################################################
if __name__ == '__main__':
    # try:


    # single time setup
    # test()
    # start rosnode
    rospy.init_node('APC_Monash')
    rate = rospy.Rate(10) # publish data at 100Hz
    rospy.loginfo("APC_Monash started")

    # start ros communications with rostopics
    ROS_Communication.ROS_Start()


    # global pub_coord_2D
    # global pub_coord_3D 
    # pub_coord_3D = rospy.Publisher("/actual_coord_3D", ActualCoord, queue_size = 10)

    while not rospy.is_shutdown():
    #try: 
        ## Start procedure
        rospy.loginfo(settings.car_coordinate_from_world)
        while settings.car_coordinate_from_world[0] ==0:      
            Movement_Control.carControl(targetSpeed = 0,steerAngle = 0)
            # rospy.loginfo(settings.car_coordinate_from_world)
            pass
        rospy.loginfo("start coord:")
        rospy.loginfo(settings.car_coordinate_from_world)

        # infinite loop
        main()
        rate.sleep()
 
    # except ValueError:
    #     print("Oops!  That was no valid number.  Try again...")

    # except rospy.ROSInterruptException: # if we stop the script (using CTRL+C), it will run rospy.ROSInterruptException
        
    #     rospy.loginfo("Exit program successful") # Exit message
    #     pass




    '''  THis
    def R1():
    Coordinate_System.travel_to2(20, [-206.4, 4.2]) # P1
    # Coordinate_System.travel_to2(20, [-245.5,0.8]) 
    # Coordinate_System.travel_to2(20, [-255.9,0.2]) # P2   #stop

    Coordinate_System.corner2(20,15,180,[-272.5,-43.9],-90) #turn left
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


    # Coordinate_System.travel_to2(20, [-219.0,-94.6]) 
    # Coordinate_System.travel_to2(20, [-205.5,-95.0]) # P4
    # Coordinate_System.travel_to2(20, [-205.3,-94.7]) # traffic light 
    # Coordinate_System.travel_to2(25, [-205.3,-91.4]) # traffic light 

def R2():
    # curve
    # Coordinate_System.travel_to2(20, [-199.1,-95.0])
    # Coordinate_System.travel_to2(25, [-195.0,-104.6]) 
    Coordinate_System.travel_to2(20, [-195.0,-118.3]) 
    Coordinate_System.travel_to2(20, [-193.9,-124.9]) 
    Coordinate_System.travel_to2(20, [-188.8,-134.2]) 
    Coordinate_System.travel_to2(20, [-183.6,-140.5]) 
    Coordinate_System.travel_to2(20, [-176.2,-146.1]) 
    Coordinate_System.travel_to2(20, [-167.5,-149.7]) 
    Coordinate_System.travel_to2(20, [-162.6,-150.6]) 
    # Coordinate_System.travel_to2(20, [-183.1,-144.5]) 
    # Coordinate_System.travel_to2(20, [-179.0,-146.9]) 
    # Coordinate_System.travel_to2(20, [-171.6,-149.4]) 
    # Coordinate_System.travel_to2(20, [-166.8,-150.3]) 
    # Coordinate_System.travel_to2(20, [-162.9,-150.7]) 
    # Coordinate_System.travel_to2(15, [-165.0,-153.9]) 

    Coordinate_System.travel_to2(20, [-158.0,-151.1]) 
    Coordinate_System.travel_to2(20, [-151.0,-151.0]) #P6 
    Coordinate_System.travel_to2(20, [-145.6,-151.0]) # traffic light 
    # Coordinate_System.travel_to2(20, [-191.6,-116.6])
    # Coordinate_System.travel_to2(20, [-191.3,-121.5])
    # Coordinate_System.travel_to2(20, [-190.2,-126.5])
    # Coordinate_System.travel_to2(20, [-188.4,-131.5])
    # Coordinate_System.travel_to2(20, [-187.1,-134.3])
    # Coordinate_System.travel_to2(20, [-185.0,-137.3])
    # Coordinate_System.travel_to2(20, [-182.6,-140.2])
    # Coordinate_System.travel_to2(20, [-178.3,-144.0])
    # Coordinate_System.travel_to2(20, [-174.2,-146.8])
    # Coordinate_System.travel_to2(20, [-169.0,-149.1])
    # Coordinate_System.travel_to2(20, [-162.6,-150.7])
    # Coordinate_System.travel_to2(20, [-155.4,-151.0])
    # Coordinate_System.travel_to2(20, [-145.9,-151.2])

    #crossing junction
    Coordinate_System.travel_to2(20, [-111.1,-151.2])

    #curve
    Coordinate_System.travel_to2(20, [-86.5,-151.2])
    Coordinate_System.travel_to2(20, [-80.7,-150.5])
    Coordinate_System.travel_to2(20, [-74.5,-148.8])
    Coordinate_System.travel_to2(20, [-69.2,-146.4])
    Coordinate_System.travel_to2(20, [-63.4,-142.7])
    Coordinate_System.travel_to2(20, [-58.8,-138.5])
    Coordinate_System.travel_to2(20, [-54.3,-133.1])
    Coordinate_System.travel_to2(20, [-51.6,-128.6])
    Coordinate_System.travel_to2(20, [-49.4,-122.9])
    Coordinate_System.travel_to2(20, [-48.2,-117.5])
    Coordinate_System.travel_to2(20, [-47.6,-112.2])
    Coordinate_System.travel_to2(20, [-47.5,-105.5])

    pass
    # curve
    # Coordinate_System.travel_to2(20, [-199.1,-95.0])
    # Coordinate_System.travel_to2(25, [-195.0,-104.6]) 
    # Coordinate_System.travel_to2(25, [-195.0,-118.3]) 
    # Coordinate_System.travel_to2(25, [-193.7,-127.2]) 
    # Coordinate_System.travel_to2(25, [-191.8,-132.4]) 
    # Coordinate_System.travel_to2(25, [-183.9,-144.2]) 
    # Coordinate_System.travel_to2(25, [-175.5,-150.1]) 
    # # Coordinate_System.travel_to2(15, [-165.0,-153.9]) 

    # Coordinate_System.travel_to2(25, [-158.0,-151.1]) 
    # Coordinate_System.travel_to2(25, [-151.0,-151.0]) #P6 
    # Coordinate_System.travel_to2(35, [-145.6,-151.0]) # traffic light 


    # Coordinate_System.travel_to2(35, [-121.0,-151.1]) 
    # Coordinate_System.travel_to2(25, [-101.4,-154.7]) # P7

    # #curve
    # Coordinate_System.travel_to2(25, [-85.8,-154.9]) 
    # Coordinate_System.travel_to2(25, [-75.0,-152.7]) 
    # Coordinate_System.travel_to2(25, [-65.8,-148.7]) 
    # Coordinate_System.travel_to2(25, [-54.4,-136.6]) 

    # Coordinate_System.travel_to2(25, [-47.8,-117.2]) # P8 
    # Coordinate_System.travel_to2(25, [-47.4,-105.9]) 

def R3():
    Coordinate_System.travel_to2(20, [-47.2,-73.2])
    Coordinate_System.travel_to2(20, [-47.2,-68.2])

    #swap lanes
    Coordinate_System.travel_to2(20, [-45.6,-52.3])
    Coordinate_System.travel_to2(20, [-43.9,-36.7])
    Coordinate_System.travel_to2(20, [-43.9,-25.0])
    Coordinate_System.travel_to2(20, [-43.90,-17.10,0.00]) #P10

    Coordinate_System.corner2(20,10,90,[-24.2,-3.1],0) #turn right
    # Coordinate_System.travel_to2(20, [-18.6,-2.70])
    Coordinate_System.travel_to2(20, [3.00,-2.70]) #P11
    # Coordinate_System.travel_to2(20, [13.7,-2.60])

    Coordinate_System.corner2(20,10,0,[32.0,19.3],90) #turn left
    Coordinate_System.travel_to2(20, [31.30,19.30]) # P15 

    # Coordinate_System.travel_to2(20, [34.6,51.60]) 
    # Coordinate_System.travel_to2(20, [36.4,72.20])

    # # cross x junction
    # Coordinate_System.travel_to2(20, [37.2,105.0])
    # Coordinate_System.travel_to2(20, [38.1,133.2])
    Coordinate_System.travel_to2(20, [34.3,122.9])
     

    # Coordinate_System.travel_to2(25, [-83.20,-87.70]) #P32
    # Coordinate_System.travel_to2(20, [-105.9,-87.8]) 
    
    # Coordinate_System.corner2(15,5,180,[-124.1,-72.3],90) #turn right
    # Coordinate_System.travel_to2(20, [-124.30,-42.40]) # P33
    # Coordinate_System.travel_to2(30, [-124.50,-17.5])
    # Coordinate_System.travel_to2(40, [-124.50,73.0])  
    

def R6():
    # Coordinate_System.travel_to2(15, [47.9,146.0]) 
    Coordinate_System.travel_to2(20, [84.70,144.10]) #P26

    #curve
    Coordinate_System.travel_to2(20, [94,142.7]) 
    Coordinate_System.travel_to2(20, [103.8,141.1]) 
    Coordinate_System.travel_to2(20, [114.3,138.8]) 
    Coordinate_System.travel_to2(20, [124.4,134.7]) 
    Coordinate_System.travel_to2(20, [132.8,129.8]) 
    Coordinate_System.travel_to2(20, [138.6,125.0]) 
    Coordinate_System.travel_to2(20, [143.1,119.7]) 
    Coordinate_System.travel_to2(20, [146.1,115.1]) 
    Coordinate_System.travel_to2(20, [148.10,112.20]) #P27 
    Coordinate_System.travel_to2(20, [150.0,106.60]) 
    Coordinate_System.travel_to2(20, [150.8,101.40]) 
    Coordinate_System.travel_to2(20, [151.3,94.4]) 
    Coordinate_System.travel_to2(20, [151.4,87.1]) 

    Coordinate_System.travel_to2(20, [151.4,13.2]) 
    Coordinate_System.corner2(20,6,-90,[141.4,2.2],-180) #turn right
    Coordinate_System.travel_to2(20, [124.70,1.9]) #P29
    # Coordinate_System.travel_to2(20, [116.5,2.2]) 

def R7():
    # Coordinate_System.travel_to2(20, [96.2,-14.9])
    Coordinate_System.travel_to2(20, [96.20,-28.60]) #P30

    #curve
    Coordinate_System.travel_to2(20, [95.70,-43.60]) 
    Coordinate_System.travel_to2(20, [95,-52]) 
    Coordinate_System.travel_to2(20, [92.8,-59.7]) 
    Coordinate_System.travel_to2(20, [88.8,-67.3]) 
    Coordinate_System.travel_to2(20, [83.4,-73.5]) 
    Coordinate_System.travel_to2(20, [76.3,-79]) 
    Coordinate_System.travel_to2(20, [65.9,-83.4]) 
    Coordinate_System.travel_to2(20, [55.2,-84.9]) 

    Coordinate_System.travel_to2(20, [45.90,-84.90]) #P14
    Coordinate_System.travel_to2(20, [45.30,-84.90]) 
    print("finish R7 route")

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
    # Coordinate_System.travel_to2(20, [154.10,177.30])  #P19

    Coordinate_System.travel_to2(20, [154.10,176.30])  #P19

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
    Coordinate_System.travel_to2(18, [-232.6,-29.3]) 

    Coordinate_System.travel_to2(15, [-232.6,28.1]) #P24 
    '''

    ''' This
    Coordinate_System.travel_to2(25, [50.0,190.2])
    Coordinate_System.travel_to2(20, [52.0,190.5])
    Coordinate_System.travel_to2(20, [54.0,190.5]) 
    Coordinate_System.travel_to2(20, [74.0,190.5])   #P18 (74.0, 190.0)

    # Cutting to the side lane 
    Coordinate_System.travel_to2(20, [95.1,193.7])
    Coordinate_System.travel_to2(20, [95.5,193.7])
    Coordinate_System.travel_to2(20, [108.0,193.7])
    Coordinate_System.travel_to2(20, [115.8,192.5])
    Coordinate_System.travel_to2(20, [122.0,191.5])
    Coordinate_System.travel_to2(20, [126.0,190.5])
    Coordinate_System.travel_to2(20, [131.5,188.8])
    Coordinate_System.travel_to2(20, [135.0,187.5])
    Coordinate_System.travel_to2(20, [140.0,185.5])
    Coordinate_System.travel_to2(20, [145.7,182.2])
    Coordinate_System.travel_to2(20, [148.1,181.0])  
    Coordinate_System.travel_to2(20, [154.1,177.3])  #P19
    Coordinate_System.travel_to2(20, [165.0,168.0])
    Coordinate_System.travel_to2(20, [174.5,158.0])
    Coordinate_System.travel_to2(20, [180.2,150.6])
    Coordinate_System.travel_to2(20, [184.1,143.5])
    Coordinate_System.travel_to2(20, [187.5,137.1])
    Coordinate_System.travel_to2(20, [190.0,130.7])
    Coordinate_System.travel_to2(20, [192.5,121.4])
    Coordinate_System.travel_to2(20, [193.7,117.3])
    Coordinate_System.travel_to2(20, [195.6,101.3])
    Coordinate_System.travel_to2(20, [189.1,72.0])   #189.3?
    Coordinate_System.travel_to2(20, [189.2,52.8])   #P20
    Coordinate_System.travel_to2(30, [189.2,50.0])
    Coordinate_System.travel_to2(30, [190.0,-4.0])
    Coordinate_System.travel_to2(30, [189.5,-80.0])
    Coordinate_System.travel_to2(30, [188.9,-104.7])
    Coordinate_System.travel_to2(30, [187.4,-116.4])
    Coordinate_System.travel_to2(30, [183.9,-129.5])
    Coordinate_System.travel_to2(30, [181.5,-135.7])
    Coordinate_System.travel_to2(30, [179.0,-140.0])
    Coordinate_System.travel_to2(30, [175.5,-146.0])
    Coordinate_System.travel_to2(30, [174.4,-148.0]) #P21
    Coordinate_System.travel_to2(30, [168.5,-156.0])
    Coordinate_System.travel_to2(30, [156.6,-168.0])
    Coordinate_System.travel_to2(30, [143.5,-176.9])
    Coordinate_System.travel_to2(30, [134.7,-180.9])
    Coordinate_System.travel_to2(30, [123.1,-185.0])
    Coordinate_System.travel_to2(30, [111.1,-187.4])
    Coordinate_System.travel_to2(30, [104.2,-187.8])
    Coordinate_System.travel_to2(40, [100.0,-187.9])
    Coordinate_System.travel_to2(40, [95.6,-187.8])
    Coordinate_System.travel_to2(40, [10.2,-187.9]) #P22
    Coordinate_System.travel_to2(40, [-145.8,-190.9]) #P23 
    Coordinate_System.travel_to2(40, [-152.0,-190.7])
    Coordinate_System.travel_to2(30, [-158.5,-189.7])
    Coordinate_System.travel_to2(30, [-166.2,-187.9])
    Coordinate_System.travel_to2(30, [-193.3,-177.6])
    Coordinate_System.travel_to2(30, [-196.6,-175.9])
    Coordinate_System.travel_to2(30, [-201.3,-172.6])
    Coordinate_System.travel_to2(30, [-205.4,-169.2])
    Coordinate_System.travel_to2(30, [-212.5,-161.2])
    Coordinate_System.travel_to2(30, [50.2,190.0][-219.7,-150.9])
    Coordinate_System.travel_to2(30, [-225.2,-141.4])
    Coordinate_System.travel_to2(30, [-229.0,-132.3])
    Coordinate_System.travel_to2(30, [-230.7,-125.9])
    Coordinate_System.travel_to2(30, [-232.4,-118.1])
    Coordinate_System.travel_to2(30, [-233.5,-111.5])
    Coordinate_System.travel_to2(30, [-233.8,-103.8])
    Coordinate_System.travel_to2(30, [-233.6,-84.3])
    Coordinate_System.travel_to2(40, [-232.6,28.1]) #P24 
    # Coordinate_System.travel_to2(35, [-233.9,98.7])
    # Coordinate_System.travel_to2(30, [-224.3,132.8])
    # Coordinate_System.travel_to2(30, [-210.5,151.2])
    # Coordinate_System.travel_to2(30, [-187.9,176.2])
    # Coordinate_System.travel_to2(30, [-176.2,182.5])
    # Coordinate_System.travel_to2(30, [-163.7,186.8])
    # Coordinate_System.travel_to2(30, [-125.8,186.5])
    # Coordinate_System.travel_to2(30, [-119.4,186.6]) #P25 
    '''

    ''' This
def R15():
    
    Coordinate_System.corner2(16,12,-180,[-187.9,31.1],90) #turn right
    Coordinate_System.travel_to2(20, [-187.9,65.8])  
    Coordinate_System.corner2(20,12,90,[-168.1,84.7],0) #turn right
    Coordinate_System.travel_to2(20, [-156.4,84.7])  
    Coordinate_System.corner2(20,15,0,[-124.40,123.80],90) #turn left
    # Coordinate_System.travel_to2(20, [-124.4,123.8])  
    # Coordinate_System.travel_to2(20, [-124.40,106.30])  #P35
    Coordinate_System.corner2(20,15,90,[-154.80,142.60],180) #turn left
    Coordinate_System.travel_to2(20, [-161.1,142.6])  
    Coordinate_System.travel_to2(20, [-168.5,141.5])  
    Coordinate_System.travel_to2(20, [-175.5,138.6])  
    Coordinate_System.travel_to2(20, [-179.8,135.8])  
    Coordinate_System.travel_to2(20, [-183.7,131.9])  
    Coordinate_System.travel_to2(20, [-187.2,127.3])  
    Coordinate_System.travel_to2(20, [-189.3,123.0])  
    Coordinate_System.travel_to2(20, [-191.4,115.3])  
    Coordinate_System.travel_to2(20, [-191.7,104.8])  

    # cross x junction
    Coordinate_System.travel_to2(20, [-191.6,74.1])  
    # Coordinate_System.travel_to2(20, [-191.5,15.2])  
    Coordinate_System.corner2(20,10,-90,[-206.4, 4.2],180) #turn right
'''