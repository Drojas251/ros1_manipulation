#include <ros/ros.h>

#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <my_robot_interfaces/PickPlaceObjAction.h>
#include <my_robot_interfaces/PickPlaceObjGoal.h>
#include <my_robot_interfaces/PickPlaceObjResult.h>
#include <my_robot_interfaces/PickPlaceObjFeedback.h>

#include "behaviortree_cpp_v3/behavior_tree.h"

#include <std_msgs/Float64.h>

class PickPlace : public BT::AsyncActionNode{
  public:

   PickPlace(const std::string& name, const BT::NodeConfiguration& config): BT::AsyncActionNode(name, config),
          _client("pick_place", true)
   {

   }

   static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<std::string>("object_name"),
            BT::InputPort<std::string>("location_name")
        };
    }

    BT::NodeStatus tick() override;

    virtual void halt() override
    {
        _aborted = true;
    }

  private:
    typedef actionlib::SimpleActionClient<my_robot_interfaces::PickPlaceObjAction> PickPlaceClient;
    PickPlaceClient _client;
    bool _aborted;

};