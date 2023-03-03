cmake_minimum_required(VERSION 3.0.2)
project(shell_simulation)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  message_generation
  nav_msgs
  geometry_msgs
  rospy
  std_msgs
  tf2_ros
  tf
)
add_message_files(
  FILES
  Score.msg
  ActualCoord.msg
)
catkin_python_setup()
generate_messages(
  DEPENDENCIES
  std_msgs
)
catkin_package(
  CATKIN_DEPENDS message_runtime
  CATKIN_DEPENDS std_msgs
  CATKIN_DEPENDS nav_msgs
  CATKIN_DEPENDS geometry_msgs
  CATKIN_DEPENDS rospy
  CATKIN_DEPENDS tf2_ros
  CATKIN_DEPENDS tf
)

# add_library(${PROJECT_NAME}
#   scripts/ROS_Communication.py
# )
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
    src/APC_Monash.py
    scripts/shell_simulation_2/Coordinate_System.py
    scripts/shell_simulation_2/Movement_Control.py
    scripts/shell_simulation_2/ROS_Communication.py
    scripts/shell_simulation_2/settings.py
    src/publish_score_topic.py
    src/efficiency.py
    src/score_test.py

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
