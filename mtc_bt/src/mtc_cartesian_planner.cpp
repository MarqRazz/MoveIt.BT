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

#include "mtc_bt/mtc_cartesian_planner.hpp"
#include "rclcpp/logger.hpp"
#include "rclcpp/logging.hpp"
#include <thread>

namespace
{
static const auto kLogger = rclcpp::get_logger("MTCCurrentState");
using namespace moveit::task_constructor;
}  // namespace

namespace mtc_bt
{
MTCCartesianPlanner::MTCCartesianPlanner(const std::string& name, const BT::NodeConfig& config)
  : SyncActionNode(name, config)
{
}

BT::NodeStatus MTCCartesianPlanner::tick()
{
  // validate input ports and values
  double max_vel_scale;
  if (!getInput<double>(kPortMaxVelScaling, max_vel_scale) && max_vel_scale <= 0.001 && max_vel_scale >= 1.0)
  {
    if (max_vel_scale < 0. && max_vel_scale >= 1.0)
      RCLCPP_WARN(kLogger, "Max Velocity Scaling Factor is not within valid range [0.001, 1.0]. Defaulting to 1.0");
    max_vel_scale = 1.0;
  }
  double max_accl_scale;
  if (!getInput<double>(kPortMaxAcclScaling, max_accl_scale) && max_accl_scale <= 0.001 && max_accl_scale >= 1.0)
  {
    if (max_accl_scale < 0. && max_accl_scale >= 1.0)
      RCLCPP_WARN(kLogger, "Max Acceleration Scaling Factor is not within valid range [0.001, 1.0]. Defaulting to 1.0");
    max_accl_scale = 1.0;
  }
  double step_size;
  if (!getInput<double>(kPortStepSize, step_size) && step_size < 0.)
  {
    RCLCPP_WARN(kLogger, "Step size is not is greater than zero. Defaulting to 0.01");
    step_size = 0.01;
  }

  // create the planner and set the output port to it
  auto planner = std::make_shared<solvers::CartesianPath>();
  planner->setMaxVelocityScalingFactor(max_vel_scale);
  planner->setMaxAccelerationScalingFactor(max_accl_scale);
  planner->setStepSize(step_size);

  setOutput(kPortPlanner, static_cast<moveit::task_constructor::solvers::PlannerInterfacePtr>(planner));
  return BT::NodeStatus::SUCCESS;
}

}  // namespace mtc_bt
