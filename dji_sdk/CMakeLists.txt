cmake_minimum_required(VERSION 2.8.3)
project(dji_sdk)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs 
  dji_sdk_lib
  geometry_msgs
  message_generation
  nav_msgs
  nodelet
  roscpp
  rospy
  std_msgs
  tf
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  Acceleration.msg
  AttitudeQuaternion.msg
  Compass.msg
  FlightControlInfo.msg
  Gimbal.msg
  GlobalPosition.msg
  LocalPosition.msg
  PowerStatus.msg
  RCChannels.msg
  TransparentTransmissionData.msg
  Velocity.msg
  Waypoint.msg
  WaypointList.msg
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  AttitudeControl.srv
  CameraActionControl.srv
  DroneTaskControl.srv
  GimbalAngleControl.srv
  GimbalSpeedControl.srv
  GlobalPositionControl.srv
  LocalPositionControl.srv
  SDKPermissionControl.srv
  SendDataToRemoteDevice.srv
  VelocityControl.srv
)

## Generate actions in the 'action' folder
add_action_files(
  FILES
  DroneTask.action
  GlobalPositionNavigation.action
  LocalPositionNavigation.action
  WaypointNavigation.action
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  actionlib_msgs
  geometry_msgs 
  nav_msgs
  std_msgs
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES dji_sdk_nodelet dji_sdk_node
  CATKIN_DEPENDS message_runtime actionlib actionlib_msgs geometry_msgs nav_msgs std_msgs tf dji_sdk_lib nodelet
  DEPENDS 
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)

include_directories(
	include
  ${catkin_INCLUDE_DIRS}
)

add_library(dji_sdk_node
  src/dji_sdk_node.cpp 
  src/modules/dji_sdk_node_actions.cpp 
  src/modules/dji_sdk_node_services.cpp 
  src/modules/dji_sdk_node_main.cpp 
)

add_dependencies(dji_sdk_node
  dji_sdk_generate_messages_cpp)

target_link_libraries(dji_sdk_node
  ${catkin_LIBRARIES}
)

add_library(dji_sdk_nodelet
  src/dji_sdk_nodelet.cpp
  src/modules/dji_sdk_node_actions.cpp 
  src/modules/dji_sdk_node_services.cpp 
  src/modules/dji_sdk_node_main.cpp 
)

add_dependencies(dji_sdk_nodelet
  dji_sdk_generate_messages_cpp)

target_link_libraries(dji_sdk_nodelet 
  ${catkin_LIBRARIES}
)


## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes


#############
## Install ##
#############
# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
install(TARGETS dji_sdk_node dji_sdk_nodelet 
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_dji_sdk.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
