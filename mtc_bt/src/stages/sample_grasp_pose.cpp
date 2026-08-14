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

#include "mtc_bt/stages/sample_grasp_pose.hpp"

namespace
{
static const auto kLogger = rclcpp::get_logger("MTCSampleGraspPose");
using namespace moveit::task_constructor;
}  // namespace

namespace mtc_bt
{
MTCSampleGraspPose::MTCSampleGraspPose(const std::string& name, const BT::NodeConfig& config) : SyncActionNode(name, config)
{
}

BT::NodeStatus MTCSampleGraspPose::tick()
{
  // these ports have defaults defined in .hpp file
  std::string stage_name, monitored_stage_name, marker_ns;
  int max_solutions;
  getInput<std::string>(kPortStageName, marker_ns);
  getInput<std::string>(kPortMonitoredStageName, monitored_stage_name);
  getInput<std::string>(kPortMarkerNs, stage_name);
  getInput<int>(kPortMaxSolutions, max_solutions);

  // validate input ports
  TaskPtr task;
  if (!getInput<TaskPtr>(kPortTask, task))
  {
    RCLCPP_ERROR(kLogger, "Missing required input: [%s]", kPortTask);
    return BT::NodeStatus::FAILURE;
  }
  geometry_msgs::msg::PoseStamped seed_pose;
  if (!getInput<geometry_msgs::msg::PoseStamped>(kPortSeedPose, seed_pose))
  {
    RCLCPP_ERROR(kLogger, "Missing required input: [%s]", kPortStageName);
    return BT::NodeStatus::FAILURE;
  }
  PoseDimension pose_dimension;
  if (!getInput<PoseDimension>(kPortPoseDimension, pose_dimension))
  {
    RCLCPP_ERROR(kLogger, "Missing required input: [%s]", kPortPoseDimension);
    return BT::NodeStatus::FAILURE;
  }
  double sampling_width;
  if (!getInput<double>(kPortSamplingWidth, sampling_width))
  {
    RCLCPP_ERROR(kLogger, "Missing required input: [%s]", kPortSamplingWidth);
    return BT::NodeStatus::FAILURE;
  }

  // create the stage and set the output port to it
  auto stage = std::make_unique<stages::GenerateRandomPose>("generate random pose");
  stage->setPose(seed_pose);
  stage->setMaxSolutions(max_solutions);
  stage->sampleDimension<std::uniform_real_distribution>(
    static_cast<stages::GenerateRandomPose::PoseDimension>(pose_dimension), sampling_width);
  stage->setMarkerNS(marker_ns);
  if (!monitored_stage_name.empty())
  {
    stage->setMonitoredStage(task->stages()->findChild(monitored_stage_name));
  }

  auto wrapper = std::make_unique<stages::ComputeIK>("pose IK", std::move(stage));
  wrapper->setMaxIKSolutions(4);
  wrapper->setMinSolutionDistance(1.0);
  wrapper->setGroup("right_manipulator");
  wrapper->setEndEffector("right_eff");
  wrapper->setIKFrame("right_arm_grasp_link");
  // get each target pose from the InterfaceState of the GenerateRandomPose stage above
  wrapper->properties().configureInitFrom(Stage::INTERFACE, { "target_pose" });

  task->add(std::move(wrapper));

  setOutput(kPortTask, task);
  return BT::NodeStatus::SUCCESS;
}

}  // namespace mtc_bt
