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

#include "moveit_msgs_bt/msg/create_collision_object.hpp"

namespace
{
static const auto kLogger = rclcpp::get_logger("CreateMoveItCollisionObject");
}  // namespace

namespace moveit_msgs_bt
{
CreateMoveItCollisionObject::CreateMoveItCollisionObject(const std::string& name, const BT::NodeConfig& config)
  : SyncActionNode(name, config)
{
}

BT::NodeStatus CreateMoveItCollisionObject::tick()
{
  // these ports have defaults defined in .hpp file
  CollisionShape primitive_type;
  getInput<CollisionShape>(kPortObjectPrimitiveType, primitive_type);
  CollisionOperation operation;
  getInput<CollisionOperation>(kPortOperation, operation);

  // validate required input ports
  std::string obj_name;
  if (!getInput<std::string>(kPortObjectName, obj_name))
  {
    RCLCPP_ERROR(kLogger, "missing required input [%s]", kPortObjectName);
    return BT::NodeStatus::FAILURE;
  }

  // create the collision object message
  moveit_msgs::msg::CollisionObject collision_obj;
  collision_obj.operation = operation;
  collision_obj.id = obj_name;
  collision_obj.header.frame_id = "world"; // if you get this wrong you get an error print but planning succeeds.
  //[moveit_planning_scene.planning_scene]: Unknown frame: base_link
	collision_obj.primitives.resize(1);
	collision_obj.primitives[0].type = primitive_type;
	collision_obj.primitives[0].dimensions = { 0.4, 0.5, 0.1 };


  geometry_msgs::msg::Pose pose;
  pose.position.x = 0.5;
  pose.position.y = -0.25;
	pose.position.z -= 0.5 * collision_obj.primitives[0].dimensions[2];  // align surface with world
	collision_obj.primitive_poses.push_back(pose);

  setOutput(kPortCollisionObject, collision_obj);
  return BT::NodeStatus::SUCCESS;
}

}  // namespace moveit_msgs_bt
