// Copyright YEAR MAINTAINER_NAME
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

#ifndef HELLO_WORLD__HELLO_WORLD_NODE_HPP_
#define HELLO_WORLD__HELLO_WORLD_NODE_HPP_

#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "hello_world/hello_world.hpp"

namespace hello_world
{
using HelloWorldPtr = std::unique_ptr<hello_world::HelloWorld>;

class HELLO_WORLD_PUBLIC HelloWorldNode : public rclcpp::Node
{
public:
  explicit HelloWorldNode(const rclcpp::NodeOptions & options);

private:
  HelloWorldPtr hello_world_{nullptr};
  void foo();
};
}  // namespace hello_world

#endif  // HELLO_WORLD__HELLO_WORLD_NODE_HPP_
