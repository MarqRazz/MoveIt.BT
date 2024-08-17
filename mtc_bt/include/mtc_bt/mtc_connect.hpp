// Copyright 2024 Marq Rasmussen
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
#include <moveit/task_constructor/stages/connect.h>
#include <moveit/task_constructor/solvers/planner_interface.h>

namespace mtc_bt
{

class MTCConnectStage : public BT::SyncActionNode
{
public:
  MTCConnectStage(const std::string& name, const BT::NodeConfig& config);

  static BT::PortsList providedPorts()
  {
    return { BT::InputPort<std::string>(kPortStageName, "ConnectStage", "default stage name"),
             BT::InputPort<std::string>(kPortGroupName, "manipulator", "default motion planning group name"),
             BT::InputPort<double>(kPortTimeout, 1.0, "default motion planning timeout to use for this stage"),
             BT::InputPort<moveit::task_constructor::solvers::PlannerInterfacePtr>(kPortMtcPlanner),
             BT::BidirectionalPort<moveit::task_constructor::TaskPtr>(kPortTask) };
  }

  BT::NodeStatus tick() override;

private:
  // Port name definitions
  static constexpr auto kPortStageName = "stage_name";
  static constexpr auto kPortGroupName = "group_name";
  static constexpr auto kPortTimeout = "timeout";
  static constexpr auto kPortMtcPlanner = "mtc_planner";
  static constexpr auto kPortTask = "task";
};

}  // namespace mtc_bt
