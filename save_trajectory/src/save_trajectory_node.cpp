// Copyright 2023 Joanna_Walowska
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

#include "save_trajectory/save_trajectory_node.hpp"

namespace save_trajectory
{

SaveTrajectoryNode::SaveTrajectoryNode(const rclcpp::NodeOptions & options)
:  Node("save_trajectory", options)
{
  save_trajectory_ = std::make_unique<save_trajectory::SaveTrajectory>();
  const int64_t param_name = this->declare_parameter("param_name", 456);
  save_trajectory_->setParameters(param_name);
  this->foo();
  subscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped>("/ground_truth/pose", 10, std::bind(&SaveTrajectoryNode::get_topic, this, std::placeholders::_1));
  
}

void SaveTrajectoryNode::get_topic(const geometry_msgs::msg::PoseStamped::SharedPtr msg) const
{
  std::cout << msg->pose.position.x << ", "<< msg->pose.position.x << std::endl;
}

void SaveTrajectoryNode::foo()
{
  // save_trajectory_->printHello();
}

}  // namespace save_trajectory

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(save_trajectory::SaveTrajectoryNode)
