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
  joy_sub = this->create_subscription<sensor_msgs::msg::Joy>("/joy", 10, std::bind(&GamepadSupportNode::joy_callback, this, std::placeholders::_1));
  steer_pub = this->create_publisher<autoware_auto_control_msgs::msg::AckermannControlCommand>("/control/command/control_cmd", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
  odom_sub = this->create_subscription<nav_msgs::msg::Odometry>("/localization/odometry", 1, std::bind(&GamepadSupportNode::odometry_callback, this, std::placeholders::_1));
}

void GamepadSupportNode::joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg) 
{

  auto left_joystick = msg->axes[0];
  auto r2_trigger = std::max(0.0f, -msg->axes[5]);
  auto l2_trigger = std::max(0.0f, -msg->axes[4]);
  auto x_button = msg->buttons[0];

  std::cout << "---------------------------------------------------------------------" << std::endl;
  std::cout << "JOYSTICK" << std::endl;
  std::cout << left_joystick << std::endl;
  std::cout << "R2" << std::endl;
  std::cout << r2_trigger << std::endl;
  std::cout << "L2" << std::endl;
  std::cout << l2_trigger << std::endl;
  std::cout << "Steering Angle: " << steer_ratio_ * left_joystick << std::endl;
  std::cout << "---------------------------------------------------------------------" << std::endl;
  

  autoware_auto_control_msgs::msg::AckermannControlCommand cmd;
  cmd.stamp = this->now();
  {
    cmd.lateral.steering_tire_angle = steer_ratio_ * left_joystick;
    cmd.lateral.steering_tire_rotation_rate = steering_angle_velocity_;

    // Backward
    if (r2_trigger && x_button) {
      cmd.longitudinal.acceleration = backward_accel_ratio_ * r2_trigger;
      cmd.longitudinal.speed =
        twist_->twist.linear.x - velocity_gain_ * cmd.longitudinal.acceleration;
      cmd.longitudinal.speed =
        std::max(cmd.longitudinal.speed, static_cast<float>(-max_backward_velocity_));
    }
    else if (r2_trigger) {
      cmd.longitudinal.acceleration = accel_ratio_ * r2_trigger;
      cmd.longitudinal.speed =
        twist_->twist.linear.x + velocity_gain_ * cmd.longitudinal.acceleration;
      cmd.longitudinal.speed =
        std::min(cmd.longitudinal.speed, static_cast<float>(max_forward_velocity_));
    }

    if (l2_trigger) {
      cmd.longitudinal.speed = 0.0;
      cmd.longitudinal.acceleration = -brake_ratio_ * l2_trigger;
    }

 
  }
  

  prev_control_command_ = cmd;
  steer_pub->publish(cmd);
}

void GamepadSupportNode::odometry_callback(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
{
  auto twist = std::make_shared<geometry_msgs::msg::TwistStamped>();
  twist->header = msg->header;
  twist->twist = msg->twist.twist;

  twist_ = twist;
}

}  // namespace gamepad_support

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(gamepad_support::GamepadSupportNode)
