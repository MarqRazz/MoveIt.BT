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

#include "mtc_bt/mtc_plan_task.hpp"

namespace
{
static const auto kLogger = rclcpp::get_logger("MTCPlanTask");
using namespace moveit::task_constructor;
}  // namespace

namespace mtc_bt
{
MTCPlanTask::MTCPlanTask(const std::string& name, const BT::NodeConfig& config) : StatefulActionNode(name, config)
{
}

BT::NodeStatus MTCPlanTask::onStart()
{
  // these ports have defaults defined in .hpp file
  getInput<int>(kPortMaxSolution, num_solutions_);
  // validate required input ports
  if (!getInput<TaskPtr>(kPortTask, task_))
  {
    RCLCPP_ERROR(kLogger, "Missing required input: [%s]", kPortTask);
    return BT::NodeStatus::FAILURE;
  }

  // make sure we can initialize the task before we attempt to plan
  try
  {
    task_->init();
  }
  catch (InitStageException& e)
  {
    RCLCPP_ERROR_STREAM(kLogger, "MTC Task Initialization failed: " << e);
    return BT::NodeStatus::FAILURE;
  }

  planning_status_ = BT::NodeStatus::RUNNING;

  // spin up a thread to plan the task
  thread_ = std::thread{ [=]() { planTask(); } };

  return planning_status_;
}

BT::NodeStatus MTCPlanTask::onRunning()
{
  return planning_status_;
}

void MTCPlanTask::onHalted()
{
  task_->preempt();
}

void MTCPlanTask::planTask()
{
  try
  {
    auto moveit_error_code = task_->plan(num_solutions_);
    if (moveit_error_code != moveit_error_code.SUCCESS)
    {
      RCLCPP_ERROR(kLogger, "MTC Task planning failed with MoveItErrorCode: %d", moveit_error_code.val);
      planning_status_ = BT::NodeStatus::FAILURE;
      return;
    }
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR_STREAM(kLogger, "MTC Task planning failed with exception: " << e.what());
    planning_status_ = BT::NodeStatus::FAILURE;
    return;
  }

  planning_status_ = BT::NodeStatus::SUCCESS;
}
}  // namespace mtc_bt
