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

#ifndef RUN_TEST__RUN_TEST_NODE_HPP_
#define RUN_TEST__RUN_TEST_NODE_HPP_

// using std::placeholders::_1;
#include <memory>
#include <chrono>
#include <string>
#include <rclcpp/rclcpp.hpp>

#include "run_test/run_test.hpp"
#include "autoware_auto_control_msgs/msg/ackermann_lateral_command.hpp"
#include "autoware_auto_control_msgs/msg/longitudinal_command.hpp"
#include "autoware_auto_control_msgs/msg/ackermann_control_command.hpp"
#include "geometry_msgs/msg/twist_with_covariance_stamped.hpp"
using namespace std::chrono_literals;

namespace run_test
{
using RunTestPtr = std::unique_ptr<run_test::RunTest>;

class RUN_TEST_PUBLIC RunTestNode : public rclcpp::Node
{
public:
  explicit RunTestNode(const rclcpp::NodeOptions & options);

private:
  RunTestPtr run_test_{nullptr};
  void foo();
  void timer_callback();

  rclcpp::Subscription<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr subscription_;
  void get_topic(const geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr msg) const;
  rclcpp::Publisher<autoware_auto_control_msgs::msg::AckermannControlCommand>::SharedPtr pub_ack;
  rclcpp::TimerBase::SharedPtr timer_;
};
}  // namespace run_test

#endif  // RUN_TEST__RUN_TEST_NODE_HPP_
