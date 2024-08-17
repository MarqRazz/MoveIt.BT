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

#include "mtc_bt/mtc_move_to.hpp"

namespace
{
static const auto kLogger = rclcpp::get_logger("MTCMoveToStage");
using namespace moveit::task_constructor;
}  // namespace

namespace mtc_bt
{
MTCMoveToStage::MTCMoveToStage(const std::string& name, const BT::NodeConfig& config) : SyncActionNode(name, config)
{
}

BT::NodeStatus MTCMoveToStage::tick()
{
  // validate input ports
  TaskPtr task;
  if (!getInput<TaskPtr>(kPortTask, task))
  {
    RCLCPP_ERROR(kLogger, "Missing required input: [%s]", kPortTask);
    return BT::NodeStatus::FAILURE;
  }
  std::string stage_name;
  if (!getInput<std::string>(kPortStageName, stage_name))
  {
    RCLCPP_ERROR(kLogger, "Missing required input: [%s]", kPortStageName);
    return BT::NodeStatus::FAILURE;
  }
  std::string group_name;
  if (!getInput<std::string>(kPortGroupName, group_name))
  {
    RCLCPP_ERROR(kLogger, "Missing required input: [%s]", kPortGroupName);
    return BT::NodeStatus::FAILURE;
  }
  std::string goal_name;
  if (!getInput<std::string>(kPortGoalName, goal_name))
  {
    RCLCPP_ERROR(kLogger, "Missing required input: [%s]", kPortGoalName);
    return BT::NodeStatus::FAILURE;
  }
  solvers::PlannerInterfacePtr mtc_planner;
  if (!getInput<solvers::PlannerInterfacePtr>(kPortMtcPlanner, mtc_planner))
  {
    RCLCPP_ERROR(kLogger, "Missing required input: [%s]", kPortMtcPlanner);
    return BT::NodeStatus::FAILURE;
  }

  // create the stage and set the output port to it
  auto stage = std::make_unique<stages::MoveTo>(stage_name, mtc_planner);
  stage->setGroup(group_name);
  stage->setGoal(goal_name);
  stage->properties().configureInitFrom(Stage::PARENT);
  std::vector<std::string> controllers = { "panda_arm_controller", "panda_hand_controller" };
  stage->properties().set("trajectory_execution_info", TrajectoryExecutionInfo().set__controller_names(controllers));
  task->add(std::move(stage));

  setOutput(kPortTask, task);
  return BT::NodeStatus::SUCCESS;
}

}  // namespace mtc_bt
