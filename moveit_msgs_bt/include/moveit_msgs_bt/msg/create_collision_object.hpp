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
#include <moveit_msgs/msg/collision_object.hpp>
#include <rclcpp/node.hpp>
#include "moveit_msgs_bt/moveit_msgs_enums.hpp"

namespace moveit_msgs_bt
{

class CreateMoveItCollisionObject : public BT::SyncActionNode
{
public:
  CreateMoveItCollisionObject(const std::string& name, const BT::NodeConfig& config);

  static BT::PortsList providedPorts()
  {
    return { BT::InputPort<std::string>(kPortObjectName, "collision_object", "default object name"),
             BT::InputPort<std::string>(kPortFrameId),
             BT::InputPort<geometry_msgs::msg::Pose>(kPortPose, geometry_msgs::msg::Pose(), "default object pose"),
             BT::InputPort<CollisionShape>(kPortObjectPrimitiveType, CollisionShape::BOX, "default object primitive shape"),
             BT::InputPort<CollisionOperation>(kPortOperation, CollisionOperation::ADD, "default operation to apply to the planning scene"),
             BT::OutputPort<moveit_msgs::msg::CollisionObject>(kPortCollisionObject) };
  }

  BT::NodeStatus tick() override;

private:
  // Port name definitions
  static constexpr auto kPortObjectName = "object_name";
  static constexpr auto kPortFrameId = "frame_id";
  static constexpr auto kPortPose = "pose";
  static constexpr auto kPortObjectPrimitiveType = "primitive_type";
  static constexpr auto kPortOperation = "operation";
  static constexpr auto kPortCollisionObject = "collision_msg";
};

}  // namespace moveit_msgs_bt
