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

#include "hello_world/hello_world_node.hpp"

namespace hello_world
{

HelloWorldNode::HelloWorldNode(const rclcpp::NodeOptions & options)
:  Node("hello_world", options)
{
  hello_world_ = std::make_unique<hello_world::HelloWorld>();
  const int64_t param_name = this->declare_parameter("param_name", 456);
  hello_world_->setParameters(param_name);
  this->foo();
}

void HelloWorldNode::foo()
{
  hello_world_->printHello();
}

}  // namespace hello_world

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(hello_world::HelloWorldNode)
