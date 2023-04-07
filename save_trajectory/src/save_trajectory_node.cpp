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
using namespace std::chrono_literals;


float x = 0.0;
float y = 0.0;
float longi = 0.0;
float latera = 0.0;
float heading = 0.0;
float steer = 0.0;
geometry_msgs::msg::Pose pose1;
autoware_auto_planning_msgs::msg::Trajectory traj;

namespace save_trajectory
{

SaveTrajectoryNode::SaveTrajectoryNode(const rclcpp::NodeOptions & options)
:  Node("save_trajectory", options)
{
  save_trajectory_ = std::make_unique<save_trajectory::SaveTrajectory>();
  const int64_t param_name = this->declare_parameter("param_name", 456);
  save_trajectory_->setParameters(param_name);
  this->foo();
  pub_ack = this->create_publisher<autoware_auto_planning_msgs::msg::Trajectory>("/trajectory", 1);

  subscription_ = this->create_subscription<nav_msgs::msg::Odometry>("/localization/odometry", 10, std::bind(&SaveTrajectoryNode::get_topic, this, std::placeholders::_1));
  subscription_vel_ = this->create_subscription<autoware_auto_vehicle_msgs::msg::VelocityReport>("/vehicle/status/velocity_status", 10, std::bind(&SaveTrajectoryNode::get_vel_topic, this, std::placeholders::_1));
  subscription_steer_ = this->create_subscription<autoware_auto_vehicle_msgs::msg::SteeringReport>("/vehicle/status/steering_status", 10, std::bind(&SaveTrajectoryNode::get_steer_topic, this, std::placeholders::_1));
  timer_ = this->create_wall_timer( 100ms, std::bind(&SaveTrajectoryNode::timer_callback, this));
}



void SaveTrajectoryNode::get_topic(const nav_msgs::msg::Odometry::SharedPtr msg) const
{
  // std::cout <<"POZYCJA: " <<msg->pose.pose.position.x << std::endl;
  pose1.position.x = msg->pose.pose.position.x;
  pose1.position.y = msg->pose.pose.position.y;
  pose1.orientation.x = msg->pose.pose.orientation.x;
  pose1.orientation.y = msg->pose.pose.orientation.y;
  // y = msg->pose.pose.position.y;
  // msg->pose.pose.position.z
}

void SaveTrajectoryNode::get_vel_topic(const autoware_auto_vehicle_msgs::msg::VelocityReport::SharedPtr msg) const
{
  // std::cout <<"VELOCITY: " << msg->longitudinal_velocity << std::endl;
  longi = msg->longitudinal_velocity;
  latera = msg->lateral_velocity;
  heading = msg->heading_rate;
}

void SaveTrajectoryNode::get_steer_topic(const autoware_auto_vehicle_msgs::msg::SteeringReport::SharedPtr msg) const
{
  // std::cout <<"STEEERING: " << msg->steering_tire_angle << std::endl;
  steer = msg->steering_tire_angle;
}

void SaveTrajectoryNode::foo()
{
  // save_trajectory_->printHello();
}

void SaveTrajectoryNode::timer_callback()
{
  autoware_auto_planning_msgs::msg::TrajectoryPoint point;
  // std::cout << x << ' ' << y << std::endl;
  point.pose = pose1;
  point.longitudinal_velocity_mps = longi;
  point.lateral_velocity_mps = latera;
  point.heading_rate_rps = heading;
  point.front_wheel_angle_rad = steer;
  traj.points.push_back(point);
  pub_ack->publish(traj);
}

}  // namespace save_trajectory

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(save_trajectory::SaveTrajectoryNode)
