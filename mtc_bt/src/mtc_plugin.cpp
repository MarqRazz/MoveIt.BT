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

#include "behaviortree_ros2/plugins.hpp"
// --- Behaviors to create MTC motion planning solvers ---
#include "mtc_bt/solvers/mtc_cartesian_planner.hpp"
#include "mtc_bt/solvers/mtc_pipeline_planner.hpp"
// --- MTC stage Behaviors ---
#include "mtc_bt/stages/mtc_connect.hpp"
#include "mtc_bt/stages/mtc_current_state.hpp"
#include "mtc_bt/stages/mtc_move_to.hpp"
#include "mtc_bt/stages/mtc_move_relative.hpp"
#include "mtc_bt/stages/sample_grasp_pose.hpp"
#include "mtc_bt/stages/generate_grasp_pose.hpp"
#include "mtc_bt/stages/grasp_object.hpp"
#include "mtc_bt/stages/release_object.hpp"
//      --- MTC Planning Scene stage Behaviors ---
#include "mtc_bt/stages/planning_scene_add_object.hpp"
// --- MTC task Behaviors ---
#include "mtc_bt/tasks/mtc_init_task.hpp"
#include "mtc_bt/tasks/mtc_plan_task.hpp"
#include "mtc_bt/tasks/mtc_execute_task.hpp"
// --- MTC enums ---
#include "mtc_bt/mtc_enums.hpp"

namespace mtc_bt
{
// Plugin registration is required to be in the .cpp file.
BT_REGISTER_ROS_NODES(factory, params)
{
  // --- Behaviors to create MTC motion planning solvers ----
  factory.registerNodeType<MTCCartesianPlanner>("MTCCartesianPlanner");
  factory.registerNodeType<MTCPipelinePlanner>("MTCPipelinePlanner", params);
  // --- MTC stage Behaviors ----
  factory.registerNodeType<MTCConnectStage>("MTCConnectStage");
  factory.registerNodeType<MTCCurrentStateStage>("MTCCurrentStateStage");
  factory.registerNodeType<MTCMoveToStage>("MTCMoveToStage");
  factory.registerNodeType<MTCMoveRelativeStage>("MTCMoveRelativeStage");
  factory.registerNodeType<MTCSampleGraspPose>("MTCSampleGraspPose");
  factory.registerNodeType<MTCGenerateGraspPose>("MTCGenerateGraspPose");
  factory.registerNodeType<MTCGraspObject>("MTCGraspObject");
  factory.registerNodeType<MTCReleaseObject>("MTCReleaseObject");
  //      --- MTC Planning Scene stage Behaviors ----
  factory.registerNodeType<MTCPlanningSceneAddObject>("MTCPlanningSceneAddObject");
  // --- MTC task Behaviors ----
  factory.registerNodeType<MTCInitializeTask>("MTCInitializeTask", params);
  factory.registerNodeType<MTCPlanTask>("MTCPlanTask");
  factory.registerNodeType<MTCExecuteTask>("MTCExecuteTask");

  factory.registerScriptingEnums<PoseDimension>();
  
}
}  // namespace mtc_bt
