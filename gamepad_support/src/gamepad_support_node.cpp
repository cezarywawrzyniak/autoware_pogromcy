// Copyright 2023 Cezary_Wawrzyniak
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "gamepad_support/gamepad_support_node.hpp"

namespace gamepad_support
{

GamepadSupportNode::GamepadSupportNode(const rclcpp::NodeOptions & options)
:  Node("gamepad_support", options)
{
  gamepad_support_ = std::make_unique<gamepad_support::GamepadSupport>();
  const int64_t param_name = this->declare_parameter("param_name", 456);
  gamepad_support_->setParameters(param_name);
  this->foo();
}

void GamepadSupportNode::foo()
{
  gamepad_support_->printHello();
}

}  // namespace gamepad_support

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(gamepad_support::GamepadSupportNode)
