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

#pragma once
#include <moveit_msgs/msg/collision_object.hpp>

namespace moveit_msgs_bt
{

enum CollisionShape { BOX=shape_msgs::msg::SolidPrimitive::BOX,
                      CONE=shape_msgs::msg::SolidPrimitive::CONE,
                      CYLINDER=shape_msgs::msg::SolidPrimitive::CYLINDER,
                      SPHERE=shape_msgs::msg::SolidPrimitive::SPHERE};

enum CollisionOperation { ADD=moveit_msgs::msg::CollisionObject::ADD,
                          APPEND=moveit_msgs::msg::CollisionObject::APPEND,
                          MOVE=moveit_msgs::msg::CollisionObject::MOVE,
                          REMOVE=moveit_msgs::msg::CollisionObject::REMOVE};

}  // namespace moveit_msgs_bt
