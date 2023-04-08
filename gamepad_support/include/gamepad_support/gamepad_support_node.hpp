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

#ifndef GAMEPAD_SUPPORT__GAMEPAD_SUPPORT_NODE_HPP_
#define GAMEPAD_SUPPORT__GAMEPAD_SUPPORT_NODE_HPP_

#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "gamepad_support/gamepad_support.hpp"

namespace gamepad_support
{
using GamepadSupportPtr = std::unique_ptr<gamepad_support::GamepadSupport>;

class GAMEPAD_SUPPORT_PUBLIC GamepadSupportNode : public rclcpp::Node
{
public:
  explicit GamepadSupportNode(const rclcpp::NodeOptions & options);

private:
  GamepadSupportPtr gamepad_support_{nullptr};
  void foo();
};
}  // namespace gamepad_support

#endif  // GAMEPAD_SUPPORT__GAMEPAD_SUPPORT_NODE_HPP_
