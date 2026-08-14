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

#include <thread>

namespace mtc_bt
{
class MTCPlanTask : public BT::StatefulActionNode
{
public:
  MTCPlanTask(const std::string& name, const BT::NodeConfig& config);

  static BT::PortsList providedPorts()
  {
    return {
      BT::BidirectionalPort<moveit::task_constructor::TaskPtr>(kPortTask),
      BT::InputPort<int>(kPortMaxSolution, 0, "Default max solutions to search for: 0"),
    };
  }

  // Behaviors have a lifetime of the running tree.
  // We need to remove from the thread before it goes out of scope.
  ~MTCPlanTask()
  {
    std::cout << "~MTCPlanTask****************************" << std::endl;
    if (thread_.joinable())
      thread_.join();
  }

  BT::NodeStatus onStart() override;

  BT::NodeStatus onRunning() override;

  void onHalted() override;

private:
  // Port name definitions
  static constexpr auto kPortTask = "task";
  static constexpr auto kPortMaxSolution = "max_solutions";

  void planTask();

  moveit::task_constructor::TaskPtr task_;
  int num_solutions_;
  BT::NodeStatus planning_status_;
  std::thread thread_;
};

}  // namespace mtc_bt
