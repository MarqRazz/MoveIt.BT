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

#include "mtc_bt/stages/release_object.hpp"

namespace
{
static const auto kLogger = rclcpp::get_logger("MTCReleaseObject");
using namespace moveit::task_constructor;
}  // namespace

namespace mtc_bt
{
MTCReleaseObject::MTCReleaseObject(const std::string& name, const BT::NodeConfig& config) : SyncActionNode(name, config)
{
}

BT::NodeStatus MTCReleaseObject::tick()
{
  // these ports have defaults defined in .hpp file
  std::string stage_name;
  getInput<std::string>(kPortStageName, stage_name);

  // validate input ports
  TaskPtr task;
  if (!getInput<TaskPtr>(kPortTask, task))
  {
    RCLCPP_ERROR(kLogger, "Missing required input: [%s]", kPortTask);
    return BT::NodeStatus::FAILURE;
  }
  std::string object_name;
  if (!getInput<std::string>(kPortObjectName, object_name))
  {
    RCLCPP_ERROR(kLogger, "Missing required input: [%s]", kPortObjectName);
    return BT::NodeStatus::FAILURE;
  }
  std::string hand_group_name;
  if (!getInput<std::string>(kPortHandGroupName, hand_group_name))
  {
    RCLCPP_ERROR(kLogger, "Missing required input: [%s]", kPortHandGroupName);
    return BT::NodeStatus::FAILURE;
  }
  std::string hand_grasp_frame;
  if (!getInput<std::string>(kPortGraspFrame, hand_grasp_frame))
  {
    RCLCPP_ERROR(kLogger, "Missing required input: [%s]", kPortGraspFrame);
    return BT::NodeStatus::FAILURE;
  }
  solvers::PlannerInterfacePtr mtc_planner;
  if (!getInput<solvers::PlannerInterfacePtr>(kPortMtcPlanner, mtc_planner))
  {
    RCLCPP_ERROR(kLogger, "Missing required input: [%s]", kPortMtcPlanner);
    return BT::NodeStatus::FAILURE;
  }


  // create the stage and set the output port to it
  {
    auto stage = std::make_unique<stages::MoveTo>("open hand", mtc_planner);
    stage->setGroup(hand_group_name);
    stage->setGoal("open");
    task->insert(std::move(stage));
  }

  {
    // Modify planning scene (w/o altering the robot's pose) to forbid touching the object after releasing it
    auto stage = std::make_unique<stages::ModifyPlanningScene>("forbid collision (hand,object)");
    stage->allowCollisions(
        object_name,
        task->getRobotModel()->getJointModelGroup(hand_group_name)->getLinkModelNamesWithCollisionGeometry(),
        true);
    task->insert(std::move(stage));
  }

  {
    auto stage = std::make_unique<stages::ModifyPlanningScene>("detach object");
    stage->detachObject(object_name, hand_grasp_frame);
    task->insert(std::move(stage));
  }

  {
    auto stage = std::make_unique<stages::ModifyPlanningScene>("remove object");
    stage->removeObject(object_name);
    task->insert(std::move(stage));
  }

  setOutput(kPortTask, task);
  return BT::NodeStatus::SUCCESS;
}

}  // namespace mtc_bt
