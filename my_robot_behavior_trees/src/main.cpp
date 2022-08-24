#include "move_rail_action.h"
#include "pick_place.h"
#include <ros/ros.h>


#include <behaviortree_cpp_v3/bt_factory.h>
#include <behaviortree_cpp_v3/loggers/bt_cout_logger.h>
#include "behaviortree_cpp_v3/behavior_tree.h"

using namespace BT;

int main(int argc, char **argv) {

  ros::init(argc, argv, "bt_node");
  ros::NodeHandle nh("~");

  // Load XML file
  std::string xml_filename;
  nh.param<std::string>("file", xml_filename, "");
  ROS_INFO("Loading XML : %s", xml_filename.c_str());

  // BehaviorTreeFactory to register our custom nodes
  BehaviorTreeFactory factory;
  factory.registerNodeType<MoveRail>("MoveRail");
  factory.registerNodeType<PickPlace>("PickPlace");

  // Create tree from loaded xml file
  auto tree = factory.createTreeFromFile(xml_filename);

  // Create a logger
  StdCoutLogger logger_cout(tree);

  NodeStatus status = NodeStatus::RUNNING;
  // Keep on ticking until you get either a SUCCESS or FAILURE state
  while (ros::ok() && status == NodeStatus::RUNNING) {
    status = tree.rootNode()->executeTick();
    // Sleep 100 milliseconds
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  return 0;
}