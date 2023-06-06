// Copyright 2023 borys
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

#ifndef SUB_TEST__SUB_TEST_NODE_HPP_
#define SUB_TEST__SUB_TEST_NODE_HPP_

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <autoware_planning_msgs/msg/lanelet_route.hpp>
#include <autoware_auto_planning_msgs/msg/trajectory_point.hpp>
#include <chrono>
#include "tier4_planning_msgs/msg/scenario.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include <algorithm>
#include <math.h>

#include "sub_test/sub_test.hpp"

namespace sub_test
{
using SubTestPtr = std::unique_ptr<sub_test::SubTest>;

class SUB_TEST_PUBLIC SubTestNode : public rclcpp::Node
{
public:
  explicit SubTestNode(const rclcpp::NodeOptions & options);

private:
  SubTestPtr sub_test_{nullptr};
  void foo();
  rclcpp::TimerBase::SharedPtr timer_;
  void timer_callback();
  mutable geometry_msgs::msg::PoseStamped point;
  mutable geometry_msgs::msg::Pose base;

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_point;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_laser_scan_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odometry_;
  void get_odometry(const nav_msgs::msg::Odometry::SharedPtr msg) const;

  void get_laser_scan(const sensor_msgs::msg::LaserScan::SharedPtr msg) const;
};
}  // namespace sub_test

#endif  // SUB_TEST__SUB_TEST_NODE_HPP_
