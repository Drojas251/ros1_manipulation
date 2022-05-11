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
  std_msgs
)
find_package(BehaviorTreeV3)


catkin_package(
   INCLUDE_DIRS
   LIBRARIES Move_Rail
   CATKIN_DEPENDS std_msgs actionlib actionlib_msgs behaviortree_cpp_v3 roscpp
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations

include_directories( ${catkin_INCLUDE_DIRS})
# include_directories(/opt/sema/include/)

# link_directories(/opt/sema/binary/linux64/lib/)

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide

add_executable(${PROJECT_NAME}_node
   src/test_bt.cpp
   src/move_rail_action.cpp
   )

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME my_nodes PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_node 
BT::behaviortree_cpp_v3
  ${catkin_LIBRARIES}
)

# add_executable(move_rail_client src/move_rail_client.cpp)
# target_link_libraries( move_rail_client ${catkin_LIBRARIES})

# install(TARGETS move_rail_action #your_other_library
#         ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#         LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#         RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

# headers in include/your_package folder
# install(DIRECTORY include/${PROJECT_NAME}/
#         DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

# install(DIRECTORY launch/
#         DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
#         PATTERN ".svn" EXCLUDE)

