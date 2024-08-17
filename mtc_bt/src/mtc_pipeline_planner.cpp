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

#include "mtc_bt/mtc_pipeline_planner.hpp"

namespace
{
static const auto kLogger = rclcpp::get_logger("MTCConnectStage");
using namespace moveit::task_constructor;
}  // namespace

namespace mtc_bt
{
MTCPipelinePlanner::MTCPipelinePlanner(const std::string& name, const BT::NodeConfig& config,
                                       const BT::RosNodeParams& params)
  : SyncActionNode(name, config), node_params_(params)
{
}

BT::NodeStatus MTCPipelinePlanner::tick()
{
  // validate the input port
  double joint_tolerance;
  if (!getInput<double>(kPortJointGoalTolerance, joint_tolerance) && joint_tolerance < 0.)
  {
    RCLCPP_WARN(kLogger, "Joint tolerance not is greater than zero. Defaulting to 1e-5.");
    joint_tolerance = 1e-5;
  }

  // create the planner and set the output port to it
  auto node = node_params_.nh.lock();
  auto planner = std::make_shared<solvers::PipelinePlanner>(node);
  planner->setProperty("goal_joint_tolerance", joint_tolerance);

  setOutput(kPortPlanner, static_cast<moveit::task_constructor::solvers::PlannerInterfacePtr>(planner));
  return BT::NodeStatus::SUCCESS;
}

}  // namespace mtc_bt
