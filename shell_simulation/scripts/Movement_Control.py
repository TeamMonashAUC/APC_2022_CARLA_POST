'''
Movement_Control (Level 2 code)
- uses ROS_Communication.py

File purpose:
	- Speed control - uses PID control to manipulate throttle pedal & brake for consistent speed
    - Steering control - Uses linear steering to steer at a certain distance given the angle
    - easier use for higher level programming
    - uses SI unit of km/h

This function main purpose is to enables 
        Movement_Control.carControl(targetSpeed = setSpeed,steerAngle= angle+settings.car_direction_from_world[2])
to work properly (not including manipulating the speed or steering angle)
'''


#################################################################################################################################################
# import other prgramming files
import settings
import ROS_Communication


#################################################################################################################################################
# Libraries used

from simple_pid import PID  # How to use https://pypi.org/project/simple-pid/


#################################################################################################################################################
'''
Function Explanation :
    Apply PID for various speed to obtain give good PID result for given speed
'''
def PID_SetSpeed(targetSpeed):

    # PID parameters setting
    pid = PID(Kp = 0.4, Ki = 0.3, Kd = 0.2, setpoint= targetSpeed)

    return pid(settings.currentCarSpeed) # apply global variable (currentCarSpeed of the car, updated from carla rostopic)






#################################################################################################################################################
#arduino map function https://cdn.arduino.cc/reference/en/language/functions/math/map/
#for arduino map function in python https://www.theamplituhedron.com/articles/How-to-replicate-the-Arduino-map-function-in-Python-for-Raspberry-Pi/


'''
Function Explanation :
    arduino map function, converts the range of the input and output     
'''
def arduino_map_function(x, in_min, in_max, out_min, out_max):
	return float((x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min)










#################################################################################################################################################
'''
Function Explanation :
    allows easy input, Speed (in m/s, can be any negative or positive number), Steer (angle of travel wanted in degree))
'''
def carControl(targetSpeed = 0, steerAngle = 0):
    # variables to be sent to carla
    car_throttle     = 0
    car_steer    	 = 0
    car_brake    	 = False
    car_reverse     = False
    car_handBrake     = False
    
    ##########################################
    # Throttle control
    calc = PID_SetSpeed(targetSpeed) # calculate PID values (negative values means need to slow down, positive values means we need to speeed up)
    
    # decide which controls should be activated
    if targetSpeed > 0: # car is moving forward
        car_reverse = False

        if calc >= 0:     
            car_throttle = abs(calc)
        else:   		 
            car_brake = abs(calc)*0.2  # 0.2 is to reduce the braking effect on the car


    elif targetSpeed < 0: # car is moving in reverse
        car_reverse = True

        if calc <= 0:    
            car_throttle = abs(calc)
        else:   		 
            car_brake = abs(calc)*0.2  # 0.2 is to reduce the braking effect on the car


    else :    # car is stationary
        car_throttle = 0
        car_brake = 1
        car_reverse = False
    

    ##########################################
    # Steering control
    car_steer = arduino_map_function(steerAngle,-settings.max_steer_angle ,settings.max_steer_angle ,-1,1) # convert range in angle to steering angle (-70,70) -> (-1,1)


    ##########################################
    # rospy.loginfo("target:%d  current:%d car_brake:%f" , targetSpeed, settings.currentCarSpeed,car_brake)
    ROS_Communication.transmit_to_carla(car_throttle, car_steer, car_brake, car_reverse, car_handBrake)