cmake_minimum_required(VERSION 2.8.3)
project(robo_navigation)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  roscpp
  rospy
  tf
  move_base_msgs
  actionlib
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES planning
  CATKIN_DEPENDS nav_msgs roscpp rospy tf move_base_msgs actionlib
#  DEPENDS system_lib
)


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

 add_executable(send_goal src/send_goal.cpp)

 target_link_libraries(send_goal
   ${catkin_LIBRARIES}
 )

