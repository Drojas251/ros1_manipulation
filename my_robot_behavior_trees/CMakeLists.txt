#cmake_minimum_required(VERSION 3.0.2)
cmake_minimum_required(VERSION 3.5)
project(my_robot_behavior_trees)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  actionlib_msgs
  actionlib
  my_robot_interfaces
  std_msgs
  behaviortree_cpp_v3
)

catkin_package(
   INCLUDE_DIRS
   LIBRARIES Move_Rail Pick_Place
   CATKIN_DEPENDS std_msgs actionlib actionlib_msgs behaviortree_cpp_v3 roscpp
)

###########
## Build ##
###########
include_directories( ${catkin_INCLUDE_DIRS})

add_executable(${PROJECT_NAME}_node
   src/main.cpp
   src/move_rail_action.cpp
   src/pick_place_action.cpp
   )

set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME my_nodes PREFIX "")

add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(
  ${PROJECT_NAME}_node 
  ${catkin_LIBRARIES}
)

