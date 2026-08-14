// Copyright 2025 Marq Rasmussen
//
// Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated
// documentation files (the "Software"), to deal in the Software without restriction, including without limitation the
// rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to
// permit persons to whom the Software is furnished to do so, subject to the following conditions: The above copyright
// notice and this permission notice shall be included in all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
// WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
// COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR
// OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

#include "behaviortree_cpp/action_node.h"
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/stages/modify_planning_scene.h>
#include <moveit/task_constructor/stages/move_to.h>
#include <moveit/task_constructor/solvers/planner_interface.h>
#include <moveit/robot_model/robot_model.hpp>

namespace mtc_bt
{

class MTCReleaseObject : public BT::SyncActionNode
{
public:
  MTCReleaseObject(const std::string& name, const BT::NodeConfig& config);

  static BT::PortsList providedPorts()
  {
    return { BT::InputPort<std::string>(kPortStageName, "MTCReleaseObject", "default stage name"),
             BT::InputPort<std::string>(kPortObjectName),
             BT::InputPort<std::string>(kPortHandGroupName),
             BT::InputPort<std::string>(kPortGraspFrame),
             BT::InputPort<moveit::task_constructor::solvers::PlannerInterfacePtr>(kPortMtcPlanner),
             BT::BidirectionalPort<moveit::task_constructor::TaskPtr>(kPortTask) };
  }

  BT::NodeStatus tick() override;

private:
  // Port name definitions
  static constexpr auto kPortStageName = "stage_name";
  static constexpr auto kPortObjectName = "object_name";
  static constexpr auto kPortHandGroupName = "hand_group_name";
  static constexpr auto kPortGraspFrame = "hand_grasp_frame";
  static constexpr auto kPortMtcPlanner = "planner";
  static constexpr auto kPortTask = "task";
};

}  // namespace mtc_bt
