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
#include "behaviortree_ros2/ros_node_params.hpp"
#include <moveit/planning_scene/planning_scene.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <moveit/planning_scene_monitor/planning_scene_monitor.hpp>
#include <rclcpp/node.hpp>

namespace moveit_bt
{

class LoadPlanningSceneFromFile : public BT::SyncActionNode
{
public:
  LoadPlanningSceneFromFile(const std::string& name, const BT::NodeConfig& config, const BT::RosNodeParams& params);

  static BT::PortsList providedPorts()
  {
    return { BT::InputPort<std::string>(kPortPackageName),
             BT::InputPort<std::string>(kPortSubdirectory),
             BT::InputPort<std::string>(kPortFileName)};
  }

  BT::NodeStatus tick() override;

private:
  // Port name definitions
  static constexpr auto kPortPackageName = "package_name";
  static constexpr auto kPortSubdirectory = "subdirectory_name";
  static constexpr auto kPortFileName = "file_name";

  BT::RosNodeParams node_params_;
};

}  // namespace moveit_bt
