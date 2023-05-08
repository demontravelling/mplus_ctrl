cmake_minimum_required(VERSION 3.0.2)
project(mplus_ctrl)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  sensor_msgs
  std_msgs
  roscpp
  rospy
  tf
  message_generation
)
 
add_service_files(
  FILES
  clear_imu.srv 
)

generate_messages(
  DEPENDENCIES
  std_msgs
) 
catkin_package(
  CATKIN_DEPENDS message_runtime	
)

 
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${console_bridge_INCLUDE_DIRS}
)

 

 

 

add_executable(mplus_ctrl
  src/mplus_ctrl.cpp
  src/utilcrc16.cpp
)
add_dependencies(mplus_ctrl ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(mplus_ctrl
  ${catkin_LIBRARIES}
)

