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

#include "mtc_bt/mtc_connect.hpp"

namespace
{
static const auto kLogger = rclcpp::get_logger("MTCConnectStage");
using namespace moveit::task_constructor;
}  // namespace

namespace mtc_bt
{
MTCConnectStage::MTCConnectStage(const std::string& name, const BT::NodeConfig& config) : SyncActionNode(name, config)
{
}

BT::NodeStatus MTCConnectStage::tick()
{
  // validate input ports
  TaskPtr task;
  if (!getInput<TaskPtr>(kPortTask, task))
  {
    throw BT::RuntimeError("missing required input [%s]", kPortTask);
  }
  std::string stage_name;
  if (!getInput<std::string>(kPortStageName, stage_name))
  {
    throw BT::RuntimeError("missing required input [%s]", kPortStageName);
  }
  std::string group_name;
  if (!getInput<std::string>(kPortGroupName, group_name))
  {
    throw BT::RuntimeError("missing required input [%s]", kPortGroupName);
  }
  double timeout;
  if (!getInput<double>(kPortTimeout, timeout))
  {
    throw BT::RuntimeError("missing required input [%s]", kPortTimeout);
  }
  solvers::PlannerInterfacePtr mtc_planner;
  if (!getInput<solvers::PlannerInterfacePtr>(kPortMtcPlanner, mtc_planner))
  {
    throw BT::RuntimeError("missing required input [%s]", kPortMtcPlanner);
  }

  // create the stage and set the output port to it
  auto stage =
      std::make_unique<stages::Connect>(stage_name, stages::Connect::GroupPlannerVector{ { group_name, mtc_planner } });
  stage->setTimeout(timeout);
  stage->properties().configureInitFrom(Stage::PARENT);
  // stage->setPathConstraints(stage_constraint);
  task->add(std::move(stage));

  setOutput(kPortTask, task);
  return BT::NodeStatus::SUCCESS;
}

}  // namespace mtc_bt
