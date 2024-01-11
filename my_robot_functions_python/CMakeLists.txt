cmake_minimum_required(VERSION 3.0.2)
project(my_robot_functions_python)

find_package(catkin REQUIRED
  rospy
  std_msgs
  moveit_ros_planning_interface
  message_generation
  actionlib_msgs
  actionlib
  my_robot_interfaces
  my_robot_ai
)

catkin_package()
include_directories()



