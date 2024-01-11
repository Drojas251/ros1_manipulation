#include <ros/ros.h>

#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <my_robot_interfaces/PickPlaceObjAction.h>
#include <my_robot_interfaces/PickPlaceObjGoal.h>
#include <my_robot_interfaces/PickPlaceObjResult.h>
#include <my_robot_interfaces/PickPlaceObjFeedback.h>

#include "behaviortree_cpp_v3/bt_factory.h"
#include "pick_place.h"
#include <string>
#include <iostream>


BT::NodeStatus PickPlace::tick() {
  // if no server is present, fail after 2 seconds
  if (!_client.waitForServer(ros::Duration(2.0))) {
    ROS_ERROR("Can't contact pick_place server");
    return BT::NodeStatus::FAILURE;
  }

  // Take the goal from the InputPort of the Node
  std::string object_name;
  std::string location_name;

  if (!getInput<std::string>("object_name", object_name)) {
    // if I can't get this, there is something wrong with your BT.
    // For this reason throw an exception instead of returning FAILURE
    throw BT::RuntimeError("missing required input [object_name]");
  }

  if (!getInput<std::string>("location_name", location_name)) {
    // if I can't get this, there is something wrong with your BT.
    // For this reason throw an exception instead of returning FAILURE
    throw BT::RuntimeError("missing required input [location_name]");
  }

  // Reset this flag
  _aborted = false;

  std::cout << std::endl << "Goal: Move the " << object_name << " object to the " << location_name << " platform" << std::endl;

  my_robot_interfaces::PickPlaceObjGoal msg;
  msg.object_name = object_name;
  msg.location_name = location_name;

  _client.sendGoal(msg);
  std::cout << "Goal Sent" << std::endl;

  while (!_aborted && !_client.waitForResult(ros::Duration(0.02))) {
    // polling at 50 Hz. No big deal in terms of CPU
  }

  if (_aborted) {
    // this happens only if method halt() was invoked
    ROS_ERROR("PickPlace aborted");
    return BT::NodeStatus::FAILURE;
  }

  if (_client.getState() != actionlib::SimpleClientGoalState::SUCCEEDED) {
    ROS_ERROR("PickPlace failed");
    return BT::NodeStatus::FAILURE;
  }

  std::cout << "Results: Successfully placed the " << object_name << " object to the " << location_name << " platform" << std::endl;

  return BT::NodeStatus::SUCCESS;
}