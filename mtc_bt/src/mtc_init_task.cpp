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

#include "mtc_bt/mtc_init_task.hpp"
// #include <moveit_task_constructor/core/include/moveit/task_constructor/trajectory_execution_info.h>
#include <moveit_task_constructor_msgs/msg/trajectory_execution_info.hpp>

namespace
{
static const auto kLogger = rclcpp::get_logger("MTCInitializeTask");
using TrajectoryExecutionInfo = moveit_task_constructor_msgs::msg::TrajectoryExecutionInfo;
}  // namespace

namespace mtc_bt
{
MTCInitializeTask::MTCInitializeTask(const std::string& name, const BT::NodeConfig& config,
                                     const BT::RosNodeParams& params)
  : SyncActionNode(name, config), node_params_(params)
{
}

BT::NodeStatus MTCInitializeTask::tick()
{
  // validate the input port
  std::string task_name;
  if (!getInput<std::string>(kPortTaskName, task_name))
  {
    RCLCPP_ERROR(kLogger, "missing required input [%s]", kPortTaskName);
    return BT::NodeStatus::FAILURE;
  }

  // create the task and set the output port to it
  auto task = std::make_shared<moveit::task_constructor::Task>();
  auto node = node_params_.nh.lock();
  task->stages()->setName(task_name);
  task->loadRobotModel(node);
  std::vector<std::string> controllers = { "panda_arm_controller", "panda_hand_controller" };
  task->setProperty("trajectory_execution_info", TrajectoryExecutionInfo().set__controller_names(controllers));

  setOutput(kPortTask, task);
  return BT::NodeStatus::SUCCESS;
}

}  // namespace mtc_bt
