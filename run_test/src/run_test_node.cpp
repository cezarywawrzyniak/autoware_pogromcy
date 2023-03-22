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


#include "run_test/run_test_node.hpp"

using namespace std::chrono_literals;
namespace run_test
{

RunTestNode::RunTestNode(const rclcpp::NodeOptions & options)
:  Node("run_test", options)
{
  run_test_ = std::make_unique<run_test::RunTest>();
  const int64_t param_name = this->declare_parameter("param_name", 456);
  run_test_->setParameters(param_name);

  pub_ack = this->create_publisher<autoware_auto_control_msgs::msg::AckermannControlCommand>("/control/command/control_cmd", 10);
  // pub_gear = this->create_publisher<autoware_auto_vehicle_msgs::msg::GearCommand>("/control/command/gear_cmd", 10);
  subscription_ = this->create_subscription<geometry_msgs::msg::TwistWithCovarianceStamped>("/sensing/vehicle_velocity_converter/twist_with_covariance", 10, std::bind(&RunTestNode::get_topic, this, std::placeholders::_1));
  timer_ = this->create_wall_timer(500ms, std::bind(&RunTestNode::timer_callback, this));
}

void RunTestNode::timer_callback()
{
  std::cout << "hello" << std::endl;
  // auto mess2 = autoware_auto_vehicle_msgs::msg::GearCommand();
  // mess2.command = 2;
  // pub_gear->publish(mess2);
  auto message = autoware_auto_control_msgs::msg::AckermannControlCommand();
  rclcpp::Time time_now = rclcpp::Clock().now();
  message.stamp = time_now;
  message.longitudinal.stamp = time_now;
  message.lateral.stamp = time_now;
  message.longitudinal.speed = 21.0;
  message.longitudinal.acceleration = 2.0;
  message.longitudinal.jerk = 0.0;
  message.lateral.steering_tire_angle = 0.5;
  message.lateral.steering_tire_rotation_rate = 0.25;
  pub_ack->publish(message);
  
}

void RunTestNode::get_topic(const geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr msg) const
{
  std::cout << msg->twist.twist.linear.x << std::endl;
}

// void RunTestNode::get_topic(const geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr msg) const
// {
//   run_test_->printHello();
// }

void RunTestNode::foo()
{
  // run_test_->printHello();

}

}  // namespace run_test

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(run_test::RunTestNode)
