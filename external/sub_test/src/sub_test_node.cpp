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

#include "sub_test/sub_test_node.hpp"

#define PI 3.14159265
using namespace std::chrono_literals;

namespace sub_test
{

SubTestNode::SubTestNode(const rclcpp::NodeOptions & options)
:  Node("sub_test", options)
{
  sub_test_ = std::make_unique<sub_test::SubTest>();
  const int64_t param_name = this->declare_parameter("param_name", 456);
  sub_test_->setParameters(param_name);
  this->foo();
  // point.position.x=0;

  sub_laser_scan_ = this->create_subscription<sensor_msgs::msg::LaserScan>("/sensing/lidar/scan", rclcpp::QoS(rclcpp::KeepLast(5)).durability_volatile().best_effort(), std::bind(&SubTestNode::get_laser_scan, this, std::placeholders::_1));
  pub_point = this->create_publisher<geometry_msgs::msg::PoseStamped>("/planning/point", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
  sub_odometry_ = this->create_subscription<nav_msgs::msg::Odometry>("/localization/odometry", 10, std::bind(&SubTestNode::get_odometry, this, std::placeholders::_1));

  timer_ = this->create_wall_timer(1s, std::bind(&SubTestNode::timer_callback, this));
  
  



}

void SubTestNode::foo()
{
  sub_test_->printHello();
}

void SubTestNode::timer_callback()
{
  pub_point->publish(point);
}

void SubTestNode::get_laser_scan(const sensor_msgs::msg::LaserScan::SharedPtr msg) const
{
  
  
  // std::cout << "hola amigo" << std::endl;
  // float tab = msg->ranges.data(); 
  // std::cout << msg->range_min << std::endl;
  // std::cout << msg->range_max << std::endl;

  tf2::Quaternion q(
        base.orientation.x,
        base.orientation.y,
        base.orientation.z,
        base.orientation.w);
  tf2::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  std::cout << "roll " << roll << "pitch " << pitch << "yaw " << yaw << std::endl;
  float dist = *max_element(msg->ranges.begin(), msg->ranges.end()) / 10;
  int idx = distance(msg->ranges.begin(),max_element(msg->ranges.begin(), msg->ranges.end()));
  float angle_increment = msg->angle_increment;
  float angle = (-2.356194496154785 + angle_increment * idx); // / PI * 180;
  // float base_angle = acos(base.orientation.w) * 2 / PI * 180;
  
  // float len_a = sqrt(base.position.x ** 2 + base.position.y ** 2);
  // float cst = sqrt( len_a ** 2 + dist ** 2 - 2 * cos(PI - angle) * len_a * dist)
  // float diff_angle = dif_angle / PI * 180;

  // if ( ((dif_angle < 90) & (dif_angle > 180)) || (dif_angle < 270) )
  // {
    point.pose.position.x = base.position.x + dist * (cos(angle)*cos(yaw) - sin(angle)*sin(yaw));
    point.pose.position.y = base.position.y + dist * (sin(angle)*cos(yaw) + cos(angle)*sin(yaw));

    // std::cout << "yaw :" << yaw << " angle :" << angle << " diff_angle :" << diff_angle << " x :" << base.position.x <<" y :" << base.position.y << std::endl;
    std::cout << " x :" << base.position.x <<" y :" << base.position.y << " dist :" << dist << std::endl;

  // }
  // else 
  // {
  //   point.pose.position.x = base.position.x + (sin(diff_angle) * dist);
  //   point.pose.position.y = base.position.y + (cos(diff_angle) * dist);
  // }

  point.header = msg->header;

  // int i = 0;
  // while(msg->ranges[i])
  // {
  //   // std::cout << msg->ranges[i] << std::endl;
  //   i++;
  // }
  // exit(0);
  


}

void SubTestNode::get_odometry(const nav_msgs::msg::Odometry::SharedPtr msg) const
{

  base = msg->pose.pose;
  
}

}  // namespace sub_test




#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(sub_test::SubTestNode)
