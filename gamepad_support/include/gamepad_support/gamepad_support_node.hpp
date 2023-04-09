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
#include "autoware_auto_control_msgs/msg/ackermann_control_command.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "nav_msgs/msg/odometry.hpp"

namespace gamepad_support
{
using GamepadSupportPtr = std::unique_ptr<gamepad_support::GamepadSupport>;

class GAMEPAD_SUPPORT_PUBLIC GamepadSupportNode : public rclcpp::Node
{
public:
  explicit GamepadSupportNode(const rclcpp::NodeOptions & options);

private:
  GamepadSupportPtr gamepad_support_{nullptr};

  double steer_ratio_ = 0.5;
  double accel_ratio_ = 3.0;

  double steering_angle_velocity_ = 0.1;
  double velocity_gain_ = 3.0;
  double max_forward_velocity_ = 20.0;

  double brake_ratio_ = 5.0;
  double backward_accel_ratio_ = 1.0;
  double max_backward_velocity_ = 3.0;
    // update_rate: 10.0
    // accel_ratio: 3.0
    // brake_ratio: 5.0
    // steer_ratio: 0.5
    // steering_angle_velocity: 0.1
    // accel_sensitivity: 1.0
    // brake_sensitivity: 1.0
    // control_command:
    //   velocity_gain: 3.0
    //   max_forward_velocity: 20.0
    //   max_backward_velocity: 3.0
    //   backward_accel_ratio: 1.0

  geometry_msgs::msg::TwistStamped::ConstSharedPtr twist_;
  autoware_auto_control_msgs::msg::AckermannControlCommand prev_control_command_;

  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub;
  rclcpp::Publisher<autoware_auto_control_msgs::msg::AckermannControlCommand>::SharedPtr steer_pub;
  void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg);
  rclcpp::Subscription<nav_msgs::msg::Odometry>::ConstSharedPtr odom_sub;
  void odometry_callback(const nav_msgs::msg::Odometry::ConstSharedPtr msg);
};
}  // namespace gamepad_support

#endif  // GAMEPAD_SUPPORT__GAMEPAD_SUPPORT_NODE_HPP_
