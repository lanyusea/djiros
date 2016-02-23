cmake_minimum_required(VERSION 2.8.3)
project(dji_sdk_read_cam)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

## is used, also find other catkin packages
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES dji_sdk_read_cam_nodelet 
  CATKIN_DEPENDS cv_bridge image_transport nodelet roscpp sensor_msgs std_msgs
  DEPENDS 
)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  sensor_msgs
  std_msgs
  nodelet
)

find_package(OpenCV REQUIRED)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

link_directories(
  /usr/local/lib 
  /usr/lib/arm-linux-gnueabihf 
  /usr/lib)

if (CMAKE_SYSTEM_PROCESSOR MATCHES "^arm")

  add_executable(dji_sdk_read_cam_node 
    src/dji_sdk_read_cam_node_main.cpp)
  
  target_link_libraries(dji_sdk_read_cam_node
    dcam
    ${catkin_LIBRARIES}
    ${OpenCV_LIBS})

  add_library(dji_sdk_read_cam_nodelet
    src/dji_sdk_read_cam_nodelet.cpp)
  
  target_link_libraries(dji_sdk_read_cam_nodelet
    dcam
    ${catkin_LIBRARIES}
    ${OpenCV_LIBS})
  
  install(TARGETS dji_sdk_read_cam_node dji_sdk_read_cam_nodelet 
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

endif()


