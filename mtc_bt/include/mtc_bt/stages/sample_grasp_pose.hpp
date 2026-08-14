// Copyright 2025 Marq Rasmussen
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
#include <moveit/task_constructor/stages/generate_random_pose.h>
#include <moveit/task_constructor/stages/compute_ik.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include "mtc_bt/mtc_enums.hpp"

namespace mtc_bt
{

class MTCSampleGraspPose : public BT::SyncActionNode
{
public:
  MTCSampleGraspPose(const std::string& name, const BT::NodeConfig& config);

  static BT::PortsList providedPorts()
  {
    return { BT::InputPort<std::string>(kPortStageName, "MTCSampleGraspPose", "default stage name"),
             BT::InputPort<geometry_msgs::msg::PoseStamped>(kPortSeedPose),
             BT::InputPort<int>(kPortMaxSolutions, "20", "default maximum number of spawned solutions"),
             BT::InputPort<PoseDimension>(kPortPoseDimension),
             BT::InputPort<double>(kPortSamplingWidth),
             BT::InputPort<std::string>(kPortMonitoredStageName, "", "default monitored stage name"),
             BT::InputPort<std::string>(kPortMarkerNs, "MTCSampleGraspPose", "default marker namespace"),
             BT::BidirectionalPort<moveit::task_constructor::TaskPtr>(kPortTask) };
  }

  BT::NodeStatus tick() override;

private:
  // Port name definitions
  static constexpr auto kPortStageName = "stage_name";
  static constexpr auto kPortSeedPose = "seed_pose";
  static constexpr auto kPortMaxSolutions = "max_solutions";
  static constexpr auto kPortPoseDimension = "pose_dimension";
  static constexpr auto kPortSamplingWidth = "sampling_width";
  static constexpr auto kPortMonitoredStageName = "monitored_stage_name";
  static constexpr auto kPortMarkerNs = "marker_ns";
  static constexpr auto kPortTask = "task";
};

}  // namespace mtc_bt
