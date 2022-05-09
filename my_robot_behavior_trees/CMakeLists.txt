cmake_minimum_required(VERSION 3.0.2)
project(my_robot_behavior_trees)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  actionlib_msgs
  actionlib
  my_robot_interfaces
  roscpp
  std_msgs
)
find_package(BehaviorTreeV3)



catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES my_robot_behavior_trees
#  CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
)



## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(move_rail_client src/move_rail_client.cpp)

target_link_libraries( 
  move_rail_client
  ${catkin_LIBRARIES}
)

