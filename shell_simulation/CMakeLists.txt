cmake_minimum_required(VERSION 3.0.2)
project(shell_simulation)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  geometry_msgs
  rospy
  std_msgs
  tf2_ros
  tf
)

catkin_package(
  CATKIN_DEPENDS std_msgs
  CATKIN_DEPENDS nav_msgs
  CATKIN_DEPENDS geometry_msgs
  CATKIN_DEPENDS rospy
  CATKIN_DEPENDS tf2_ros
  CATKIN_DEPENDS tf
)


###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  ${catkin_INCLUDE_DIRS}
)

#add_executable(car_control_master
#    src/car_control_master.cpp
#)

#target_link_libraries(car_control_master
#  ${catkin_LIBRARIES}
#)

#############
## Install ##
#############
#install(TARGETS car_control_master
#  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)
catkin_install_python(
  PROGRAMS
    scripts/APC_Monash.py
    scripts/Coordinate_System.py
    scripts/Movement_Control.py
    scripts/ROS_Communication.py
    scripts/settings.py
    
    scripts/efficiency.py
    scripts/score.py

  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY
  scripts
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
## Mark other files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY
  launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
