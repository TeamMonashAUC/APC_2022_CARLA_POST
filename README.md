# APC_2022_CARLA_POST
## Used for Post APC 2022 competition research project

## Install Instructions
1) Git clone this repository in the path ~/carla-ros-bridge/catkin_ws/src/
2) type in the terminal "pip install simple-pid" to install simple-PID


## Program Notes:
The new research code developed is under APC_Monash folder
- This new node will stop the need of test.py as a communication bridge between carla & APC_Monash
- Reduce number of classes to make things simpler (no repeating "self." :p)
- PID implementation for throttle with variable speed using a single function 
    "Movement_Control.carControl(targetSpeed = 10,steerAngle= 0)"  
- Cleaner code for developing & understanding 

## To-do list
### Mandatory features
- [x] PID control   (Andrew)
- [x] Linear steering (Andrew)
- [x] Coordinate system for car travel
    - [x] Straight line (Andrew)
    - [x] Corners 90    (Andrew & Nulan)
    - [x] Corners any angle (Andrew & Nulan)
    - [ ] Corners using many smaller waypoints (Nulan)
- [ ] Stop when detect moving obstacle (using camera or radar) (Lucas & Qi Shiun & Guan Yuan)
    - [ ] Traffic light (not needeed for APC2023)
    - [ ] pedestrian (not needeed for APC2023)
    - [ ] car
- [ ] Lane centering using camera (opencv2)   (Nulan &  Derrick)
    - [ ] Straight line
    - [ ] Corners
- [ ] Energy Saving   
    - [ ] Soft accelerate
    - [ ] Coasting on straights
    - [ ] Coasting on corners
- [ ] Route Planning (Omar)
     
### Current issues (found during testing)
- [ ] Long sweeping turns
- [ ] Roundabout
- [ ] try to reduce tolerance needed to goals (as of now it is 1m)

### Add on features
- [ ] Map system analysis using (OpenDRIVE standard 1.4) 
    - [ ] car can automatically decide which lane to use
- [ ] Smart path planning using waypoint (make many smaller coordinate for the car to follow)
- [ ] Lidar SLAM research 

## Carla-Town03 Map (what we will be testing on prior to APC2023)
![Carla Town03 Map](https://carla.readthedocs.io/en/latest/img/Town03.jpg)



### pro tips: (https://blog.unosquare.com/10-tips-for-writing-cleaner-code-in-any-programming-language)
1) documentation is important (link sources used in learning these techniques)
2) it is okay to use longer names, VSC (Visual studio code) has autofill, it's more important to use words that we can understand easily in our code (this reduces the need for comments as well)
3) number of code lines doesn't matter, use as much as possible to make your code look easy to read
4) use more functions, to make code easier to debug  
5) We take more time reading code than writing them, so spend more in making it easy to read (make it easy for you and me :D)


### git practices:
1) when commiting, write descriptive messages to tell what you improved on and what's on your mind at that moment (example: git commit -m "README.md has been updated with instructions on folder names & tips when developing as a team")


## Older files are located in the older_ref folder for reference when developing
1) old_car_control_master.py   -   2022 APC submission code
2) car_control_master.py       -   is the modified version of the competition code (made to improve putting new coordinate locations using functions instead of super long arrays) 



###### written by : Andrew Joseph Ng
