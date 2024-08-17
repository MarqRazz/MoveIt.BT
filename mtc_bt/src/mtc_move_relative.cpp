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

#include "mtc_bt/mtc_move_relative.hpp"
#include <moveit_task_constructor_msgs/msg/trajectory_execution_info.hpp>

namespace
{
static const auto kLogger = rclcpp::get_logger("MTCMoveRelativeStage");
using namespace moveit::task_constructor;
}  // namespace

namespace mtc_bt
{
MTCMoveRelativeStage::MTCMoveRelativeStage(const std::string& name, const BT::NodeConfig& config)
  : SyncActionNode(name, config)
{
}

BT::NodeStatus MTCMoveRelativeStage::tick()
{
  // these ports have defaults defined in .hpp file
  std::string stage_name, group_name, marker_ns, direction_frame_id;
  double max_distance;
  getInput<std::string>(kPortStageName, stage_name);
  getInput<std::string>(kPortGroupName, group_name);
  getInput<double>(kPortMaxDistance, max_distance);
  getInput<std::string>(kPortMarkerNs, marker_ns);
  getInput<std::string>(kPortDirectionFrameId, direction_frame_id);
  // validate required input ports
  TaskPtr task;
  if (!getInput<TaskPtr>(kPortTask, task))
  {
    RCLCPP_ERROR(kLogger, "Missing required input: [%s]", kPortTask);
    return BT::NodeStatus::FAILURE;
  }
  moveit::task_constructor::solvers::PlannerInterfacePtr planner;
  if (!getInput<moveit::task_constructor::solvers::PlannerInterfacePtr>(kPortPlanner, planner))
  {
    RCLCPP_ERROR(kLogger, "Missing required input: [%s]", kPortPlanner);
    return BT::NodeStatus::FAILURE;
  }
  double min_distance;
  if (!getInput<double>(kPortMinDistance, min_distance))
  {
    RCLCPP_ERROR(kLogger, "missing required input [%s]", kPortMinDistance);
    return BT::NodeStatus::FAILURE;
  }
  std::string ik_frame;
  if (!getInput<std::string>(kPortIkFrame, ik_frame))
  {
    RCLCPP_ERROR(kLogger, "missing required input [%s]", kPortIkFrame);
    return BT::NodeStatus::FAILURE;
  }

  // Create the MoveRelative stage and fill everything in
  auto stage = std::make_unique<stages::MoveRelative>(stage_name, planner);
  stage->setGroup(group_name);
  stage->setMinMaxDistance(min_distance, max_distance);
  stage->setIKFrame(ik_frame);
  stage->properties().set("marker_ns", marker_ns);

  // Set upward direction
  geometry_msgs::msg::Vector3Stamped vec;
  vec.header.frame_id = direction_frame_id;
  vec.vector.z = 1.0;
  stage->setDirection(vec);
  std::vector<std::string> controllers = { "panda_arm_controller", "panda_hand_controller" };
  stage->properties().set("trajectory_execution_info", TrajectoryExecutionInfo().set__controller_names(controllers));
  task->insert(std::move(stage));

  setOutput(kPortTask, task);
  return BT::NodeStatus::SUCCESS;
}

}  // namespace mtc_bt
