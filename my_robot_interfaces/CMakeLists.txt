cmake_minimum_required(VERSION 3.0.2)
project(my_robot_interfaces)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

find_package(catkin REQUIRED
  std_msgs
  geometry_msgs
  moveit_msgs
  message_generation
  actionlib_msgs
  actionlib
)

add_action_files(
  FILES
  MoveRail.action
  PickPlaceObj.action
)

generate_messages(
 DEPENDENCIES
 std_msgs
 geometry_msgs
 moveit_msgs
 actionlib_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES my_robot_interfaces
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)


## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
# ${catkin_INCLUDE_DIRS}
)


