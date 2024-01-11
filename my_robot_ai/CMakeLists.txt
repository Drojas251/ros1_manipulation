cmake_minimum_required(VERSION 3.0.2)
project(my_robot_ai)

find_package(catkin REQUIRED COMPONENTS
  rospy
  my_robot_interfaces
)

#catkin_python_setup()
catkin_package()



include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
