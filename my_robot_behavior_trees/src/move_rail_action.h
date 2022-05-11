#include <ros/ros.h>

#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <my_robot_interfaces/MoveRailAction.h>
#include <my_robot_interfaces/MoveRailGoal.h>
#include <my_robot_interfaces/MoveRailResult.h>
#include <my_robot_interfaces/MoveRailFeedback.h>

#include "behaviortree_cpp_v3/behavior_tree.h"

#include <std_msgs/Float64.h>

class MoveRail : public BT::AsyncActionNode{
  public:

   MoveRail(const std::string& name, const BT::NodeConfiguration& config): BT::AsyncActionNode(name, config),
          _client("move_rail", true)
   {

   }

   static BT::PortsList providedPorts()
    {
        return{ BT::InputPort<double>("rail_position") };
    }

    BT::NodeStatus tick() override;

    virtual void halt() override
    {
        _aborted = true;
    }

  private:
    typedef actionlib::SimpleActionClient<my_robot_interfaces::MoveRailAction> MoveRailClient;
    MoveRailClient _client;
    bool _aborted;

};