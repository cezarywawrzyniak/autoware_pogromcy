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

#ifndef SAVE_TRAJECTORY__SAVE_TRAJECTORY_NODE_HPP_
#define SAVE_TRAJECTORY__SAVE_TRAJECTORY_NODE_HPP_

#include <memory>
#include <iostream>
#include <fstream>
#include <rclcpp/rclcpp.hpp>
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "save_trajectory/save_trajectory.hpp"

namespace save_trajectory
{
using SaveTrajectoryPtr = std::unique_ptr<save_trajectory::SaveTrajectory>;

class SAVE_TRAJECTORY_PUBLIC SaveTrajectoryNode : public rclcpp::Node
{
public:
  explicit SaveTrajectoryNode(const rclcpp::NodeOptions & options);
  
private:
  SaveTrajectoryPtr save_trajectory_{nullptr};
  void foo();
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subscription_;
  void get_topic(const geometry_msgs::msg::PoseStamped::SharedPtr msg) const;
};
}  // namespace save_trajectory

#endif  // SAVE_TRAJECTORY__SAVE_TRAJECTORY_NODE_HPP_
