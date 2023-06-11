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

#ifndef STEER_THE_CAR__STEER_THE_CAR_NODE_HPP_
#define STEER_THE_CAR__STEER_THE_CAR_NODE_HPP_

#include <memory>
#include <iostream>
#include <fstream>
#include <rclcpp/rclcpp.hpp>
#include <sstream>
#include <string>
#include <vector>
#include <cmath>
#include <tuple>
#include <algorithm>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/buffer.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "steer_the_car/steer_the_car.hpp"
#include "autoware_auto_control_msgs/msg/ackermann_control_command.hpp"
#include "autoware_auto_vehicle_msgs/msg/velocity_report.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "visualization_msgs/msg/marker.hpp"

namespace steer_the_car
{
using SteerTheCarPtr = std::unique_ptr<steer_the_car::SteerTheCar>;

class STEER_THE_CAR_PUBLIC SteerTheCarNode : public rclcpp::Node
{
public:
  explicit SteerTheCarNode(const rclcpp::NodeOptions & options);

private:
  SteerTheCarPtr steer_the_car_{nullptr};
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_marker;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_marker_list;
  rclcpp::Publisher<autoware_auto_control_msgs::msg::AckermannControlCommand>::SharedPtr steer_pub;

  rclcpp::Subscription<autoware_auto_vehicle_msgs::msg::VelocityReport>::SharedPtr vel_sub;
  void get_vel_topic(const autoware_auto_vehicle_msgs::msg::VelocityReport::SharedPtr msg);

  void pub_trajectory();
  void pub_arrow();
  visualization_msgs::msg::Marker marker2; //for line marker, whole trajectory
  visualization_msgs::msg::Marker marker; //for arrow marker

  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

  std::vector<std::tuple<double, double>> coordinates_list;
  size_t list_size;
  size_t list_index = 0;
  std::string file_path = "/home/czarek/autoware/trajectory.txt";

  double cur_point_x, cur_point_y;
  double cur_car_x, cur_car_y, cur_car_z;
  double dx, dy, distance;

  double l_d = 2.0;
  double min_ld = 1.0;
  double max_ld = 2.0;
  double K_dd = 1.0;
  double wheel_base = 0.324;
  
  double longitudinal_vel_;
  double lateral_vel_;
  double heading_rate_;

  bool first_loop = true;

  double steer_ratio_ = 0.5;
  double accel_ratio_ = 3.0;

  double steering_angle_velocity = 10.0;
  double velocity_gain_ = 3.0;
  double max_forward_velocity_ = 20.0;

  double brake_ratio_ = 5.0;
  double backward_accel_ratio_ = 1.0;
  double max_backward_velocity_ = 3.0;

  int idd = 0;

  autoware_auto_control_msgs::msg::AckermannControlCommand prev_control_command_;
};
}  // namespace steer_the_car

#endif  // STEER_THE_CAR__STEER_THE_CAR_NODE_HPP_
