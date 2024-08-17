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
#include <moveit/task_constructor/solvers/cartesian_path.h>
#include <moveit/task_constructor/solvers/planner_interface.h>

namespace mtc_bt
{

class MTCCartesianPlanner : public BT::SyncActionNode
{
public:
  MTCCartesianPlanner(const std::string& name, const BT::NodeConfig& config);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<double>(kPortMaxVelScaling, 1.0,
                            "The max velocity scaling factor to use when timing the trajectory. Valid range [0.0-1.0]"),
      BT::InputPort<double>(kPortMaxAcclScaling, 1.0,
                            "The max acceleration scaling factor to use when timing the trajectory. Valid range "
                            "[0.0-1.0]"),
      BT::InputPort<double>(kPortStepSize, 0.01,
                            "The max distance between waypoints in the Cartesian trajectory (meters)"),
      BT::OutputPort<moveit::task_constructor::solvers::PlannerInterfacePtr>(
          kPortPlanner, "{cartesian_planner}", "An MTC Motion Planner that can be used to find plans in Cartesian space")
    };
  }

  BT::NodeStatus tick() override;

private:
  // Port name definitions
  static constexpr auto kPortMaxVelScaling = "max_velocity_scaling_factor";
  static constexpr auto kPortMaxAcclScaling = "max_acceleration_scaling_factor";
  static constexpr auto kPortStepSize = "step_size";
  static constexpr auto kPortPlanner = "planner";
};

}  // namespace mtc_bt
