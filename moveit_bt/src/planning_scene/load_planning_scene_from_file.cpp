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

#include "moveit_bt/planning_scene/load_planning_scene_from_file.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>

namespace
{
static const auto kLogger = rclcpp::get_logger("LoadPlanningSceneFromFile");
}  // namespace

namespace moveit_bt
{
LoadPlanningSceneFromFile::LoadPlanningSceneFromFile(const std::string& name, const BT::NodeConfig& config,
                                                     const BT::RosNodeParams& params)
  : SyncActionNode(name, config), node_params_(params)
{
}

BT::NodeStatus LoadPlanningSceneFromFile::tick()
{
  // validate the input ports
  std::string package_name;
  if (!getInput<std::string>(kPortPackageName, package_name))
  {
    RCLCPP_ERROR(kLogger, "missing required input [%s]", kPortPackageName);
    return BT::NodeStatus::FAILURE;
  }
  std::string subdirectory;
  if (!getInput<std::string>(kPortSubdirectory, subdirectory))
  {
    RCLCPP_ERROR(kLogger, "missing required input [%s]", kPortSubdirectory);
    return BT::NodeStatus::FAILURE;
  }
  std::string file_name;
  if (!getInput<std::string>(kPortFileName, file_name))
  {
    RCLCPP_ERROR(kLogger, "missing required input [%s]", kPortFileName);
    return BT::NodeStatus::FAILURE;
  }

  std::string file_path;
  try
  {
    file_path = ament_index_cpp::get_package_share_directory(package_name) + "/" + subdirectory;
    RCLCPP_DEBUG(kLogger, "Searching for planning scene file in path: %s", file_path.c_str());
  }
  catch(const std::exception& e)
  {
    RCLCPP_ERROR(kLogger, "Failed to find package: %s \n %s", package_name.c_str(), e.what());
  }

  auto node = node_params_.nh.lock();
  robot_model_loader::RobotModelLoader::Options opt;
  opt.robot_description = "robot_description";
  opt.load_kinematics_solvers = false;
  auto rml = std::make_shared<robot_model_loader::RobotModelLoader>(node, opt);
  planning_scene::PlanningScene ps(rml->getModel());

  std::ifstream f(file_path + "/" + file_name);
  if (ps.loadGeometryFromStream(f))
  {
    RCLCPP_INFO(kLogger, "Publishing geometry from '%s'", file_name.c_str());
    moveit::planning_interface::PlanningSceneInterface psi;
    moveit_msgs::msg::PlanningScene ps_msg;
    ps.getPlanningSceneMsg(ps_msg);
    ps_msg.is_diff = true;
    ps_msg.robot_state.joint_state = sensor_msgs::msg::JointState();
    ps_msg.robot_state.multi_dof_joint_state = sensor_msgs::msg::MultiDOFJointState();
    ps_msg.robot_state.is_diff = true;
    psi.applyPlanningScene(ps_msg);
  }
  else
  {
    RCLCPP_ERROR(kLogger,"Failed to load planning scene geometry from file: %s", (file_path + "/" + file_name).c_str());
    return BT::NodeStatus::FAILURE;
  }
  return BT::NodeStatus::SUCCESS;
}


}  // namespace moveit_bt
