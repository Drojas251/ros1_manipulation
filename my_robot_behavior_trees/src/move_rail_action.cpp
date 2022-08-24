#include <ros/ros.h>

#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <my_robot_interfaces/MoveRailAction.h>
#include <my_robot_interfaces/MoveRailGoal.h>
#include <my_robot_interfaces/MoveRailResult.h>
#include <my_robot_interfaces/MoveRailFeedback.h>

#include "behaviortree_cpp_v3/bt_factory.h"
#include "move_rail_action.h"


BT::NodeStatus MoveRail::tick() {
  // if no server is present, fail after 2 seconds
  if (!_client.waitForServer(ros::Duration(2.0))) {
    ROS_ERROR("Can't contact move_base server");
    return BT::NodeStatus::FAILURE;
  }

  // Check input
  double rail_position;
  if (!getInput<double>("rail_position", rail_position)) {
    throw BT::RuntimeError("missing required input [goal]");
  }

  // Reset this flag
  _aborted = false;

  // Sending Goal
  my_robot_interfaces::MoveRailGoal msg;
  msg.position = rail_position;

  _client.sendGoal(msg);
  ROS_INFO("Goal Sent");

  while (!_aborted && !_client.waitForResult(ros::Duration(0.02))) {
    // polling at 50 Hz. No big deal in terms of CPU
  }

  if (_aborted) {
    // this happens only if method halt() was invoked
    ROS_ERROR("MoveRail aborted");
    return BT::NodeStatus::FAILURE;
  }

  if (_client.getState() != actionlib::SimpleClientGoalState::SUCCEEDED) {
    ROS_ERROR("MoveRail failed");
    return BT::NodeStatus::FAILURE;
  }

  ROS_INFO("Target rail position reached");
  return BT::NodeStatus::SUCCESS;
}