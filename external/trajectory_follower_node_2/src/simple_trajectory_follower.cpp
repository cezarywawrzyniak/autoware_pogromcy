//  Copyright 2022 Tier IV, Inc. All rights reserved.
//
//  Licensed under the Apache License, Version 2.0 (the "License");
//  you may not use this file except in compliance with the License.
//  You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
//  Unless required by applicable law or agreed to in writing, software
//  distributed under the License is distributed on an "AS IS" BASIS,
//  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//  See the License for the specific language governing permissions and
//  limitations under the License.

#include "trajectory_follower_node/simple_trajectory_follower.hpp"

#include <motion_utils/motion_utils.hpp>
#include <tier4_autoware_utils/tier4_autoware_utils.hpp>

#include <algorithm>

namespace simple_trajectory_follower
{

using motion_utils::findNearestIndex;
using tier4_autoware_utils::calcLateralDeviation;
using tier4_autoware_utils::calcYawDeviation;

SimpleTrajectoryFollower::SimpleTrajectoryFollower(const rclcpp::NodeOptions & options)
: Node("simple_trajectory_follower", options)
{
  pub_cmd_ = create_publisher<AckermannControlCommand>("/control/command/control_cmd", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());

  sub_kinematics_ = create_subscription<Odometry>(
    "/localization/odometry", 1, [this](const Odometry::SharedPtr msg) { odometry_ = msg; });
  sub_trajectory_ = create_subscription<Trajectory>(
    "/trajectory", 1, [this](const Trajectory::SharedPtr msg) { trajectory_ = msg; });

  use_external_target_vel_ = declare_parameter<bool>("use_external_target_vel", false);
  external_target_vel_ = declare_parameter<float>("external_target_vel", 0.0);
  lateral_deviation_ = declare_parameter<float>("lateral_deviation", 0.0);

  using namespace std::literals::chrono_literals;
  timer_ = rclcpp::create_timer(
    this, get_clock(), 30ms, std::bind(&SimpleTrajectoryFollower::onTimer, this));
}

void SimpleTrajectoryFollower::onTimer()
{
  if (!checkData()) {
    RCLCPP_INFO(get_logger(), "data not ready");
    return;
  }

  updateClosest();

  AckermannControlCommand cmd;
  cmd.stamp = cmd.lateral.stamp = cmd.longitudinal.stamp = get_clock()->now();
  cmd.lateral.steering_tire_angle = static_cast<float>(calcSteerCmd());
  cmd.longitudinal.speed = use_external_target_vel_ ? static_cast<float>(external_target_vel_)
                                                    : closest_traj_point_.longitudinal_velocity_mps;
  cmd.longitudinal.acceleration = static_cast<float>(calcAccCmd());
  pub_cmd_->publish(cmd);
}

void SimpleTrajectoryFollower::updateClosest()
{
  const auto closest = findNearestIndex(trajectory_->points, odometry_->pose.pose.position);
  closest_traj_point_ = trajectory_->points.at(closest);
}

double SimpleTrajectoryFollower::calcSteerCmd()
{
  const auto lat_err =
    calcLateralDeviation(closest_traj_point_.pose, odometry_->pose.pose.position) -
    lateral_deviation_;
  const auto yaw_err = calcYawDeviation(closest_traj_point_.pose, odometry_->pose.pose);

  // linearized pure_pursuit control
  constexpr auto wheel_base = 4.0;
  constexpr auto lookahead_time = 3.0;
  constexpr auto min_lookahead = 3.0;
  const auto lookahead = min_lookahead + lookahead_time * std::abs(odometry_->twist.twist.linear.x);
  const auto kp = 2.0 * wheel_base / (lookahead * lookahead);
  const auto kd = 2.0 * wheel_base / lookahead;

  constexpr auto steer_lim = 0.6;

  const auto steer = std::clamp(-kp * lat_err - kd * yaw_err, -steer_lim, steer_lim);
  RCLCPP_DEBUG(
    get_logger(), "kp = %f, lat_err = %f, kd - %f, yaw_err = %f, steer = %f", kp, lat_err, kd,
    yaw_err, steer);
  return steer;
}

double SimpleTrajectoryFollower::calcAccCmd()
{
  const auto traj_vel = static_cast<double>(closest_traj_point_.longitudinal_velocity_mps);
  const auto ego_vel = odometry_->twist.twist.linear.x;
  const auto target_vel = use_external_target_vel_ ? external_target_vel_ : traj_vel;
  const auto vel_err = ego_vel - target_vel;

  // P feedback
  constexpr auto kp = 0.5;
  constexpr auto acc_lim = 2.0;

  const auto acc = std::clamp(-kp * vel_err, -acc_lim, acc_lim);
  RCLCPP_DEBUG(get_logger(), "vel_err = %f, acc = %f", vel_err, acc);
  return acc;
}

bool SimpleTrajectoryFollower::checkData() { return (trajectory_ && odometry_); }

}  // namespace simple_trajectory_follower

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(simple_trajectory_follower::SimpleTrajectoryFollower)
