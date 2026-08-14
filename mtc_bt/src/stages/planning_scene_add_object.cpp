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

#include "mtc_bt/stages/planning_scene_add_object.hpp"

namespace
{
static const auto kLogger = rclcpp::get_logger("MTCPlanningSceneAddObject");
using namespace moveit::task_constructor;
}  // namespace

namespace mtc_bt
{
MTCPlanningSceneAddObject::MTCPlanningSceneAddObject(const std::string& name, const BT::NodeConfig& config)
  : SyncActionNode(name, config)
{
}

BT::NodeStatus MTCPlanningSceneAddObject::tick()
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
  moveit_msgs::msg::CollisionObject collision_obj_msg;
  if (!getInput<moveit_msgs::msg::CollisionObject>(kPortCollisionObjMsg, collision_obj_msg))
  {
    RCLCPP_ERROR(kLogger, "Missing required input: [%s]", kPortCollisionObjMsg);
    return BT::NodeStatus::FAILURE;
  }


  auto stage = std::make_unique<stages::ModifyPlanningScene>(stage_name);
  stage->addObject(collision_obj_msg);
  task->add(std::move(stage));

  setOutput(kPortTask, task);
  return BT::NodeStatus::SUCCESS;
}

}  // namespace mtc_bt
