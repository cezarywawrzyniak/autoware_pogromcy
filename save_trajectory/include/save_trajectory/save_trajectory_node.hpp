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
#include "autoware_auto_planning_msgs/msg/trajectory.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "save_trajectory/save_trajectory.hpp"
#include "autoware_auto_vehicle_msgs/msg/velocity_report.hpp"
#include "autoware_auto_vehicle_msgs/msg/steering_report.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <chrono>
#include "autoware_auto_planning_msgs/msg/trajectory_point.hpp"
#include "builtin_interfaces/msg/duration.hpp"
#include <rosbag2_cpp/writer.hpp>
#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include <nlohmann/json.hpp>


using json = nlohmann::json;
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
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
  rclcpp::Subscription<autoware_auto_vehicle_msgs::msg::VelocityReport>::SharedPtr subscription_vel_;
  rclcpp::Subscription<autoware_auto_vehicle_msgs::msg::SteeringReport>::SharedPtr subscription_steer_;
  rclcpp::Publisher<autoware_auto_planning_msgs::msg::Trajectory>::SharedPtr pub_ack;
  void get_topic(const nav_msgs::msg::Odometry::SharedPtr msg) const;
  void get_vel_topic(const autoware_auto_vehicle_msgs::msg::VelocityReport::SharedPtr msg) const;
  void get_steer_topic(const autoware_auto_vehicle_msgs::msg::SteeringReport::SharedPtr msg) const;
  rclcpp::TimerBase::SharedPtr timer_;
  void timer_callback();
  std::unique_ptr<rosbag2_cpp::Writer> writer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
};
}  // namespace save_trajectory

#endif  // SAVE_TRAJECTORY__SAVE_TRAJECTORY_NODE_HPP_
