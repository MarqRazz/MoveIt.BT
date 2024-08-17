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

#include "mtc_bt/mtc_current_state.hpp"

namespace
{
static const auto kLogger = rclcpp::get_logger("MTCCurrentStateStage");
using namespace moveit::task_constructor;
}  // namespace

namespace mtc_bt
{
MTCCurrentStateStage::MTCCurrentStateStage(const std::string& name, const BT::NodeConfig& config)
  : SyncActionNode(name, config)
{
}

BT::NodeStatus MTCCurrentStateStage::tick()
{
  // validate input ports
  TaskPtr task;
  if (!getInput<TaskPtr>(kPortTask, task))
  {
    RCLCPP_ERROR(kLogger, "Missing required input: [%s]", kPortTask);
    return BT::NodeStatus::FAILURE;
  }

  // create the stage and set the output port to it
  auto current_state = std::make_unique<stages::CurrentState>("Current State");

  // Verify that the start state is not in collision
  auto applicability_filter =
      std::make_unique<stages::PredicateFilter>("Start State collision check", std::move(current_state));
  applicability_filter->setPredicate([](const SolutionBase& s, std::string& comment) {
    std::vector<std::string> links;
    s.start()->scene()->getCollidingLinks(links);
    if (!links.empty())
    {
      comment = "The following links are in collision at the start state:";
      for (const auto& link : links)
      {
        comment += link + ", ";
      }
      return false;
    }
    return true;
  });

  task->add(std::move(applicability_filter));

  setOutput(kPortTask, task);
  return BT::NodeStatus::SUCCESS;
}

}  // namespace mtc_bt
