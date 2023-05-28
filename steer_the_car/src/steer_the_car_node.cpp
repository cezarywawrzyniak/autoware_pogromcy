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

#include "steer_the_car/steer_the_car_node.hpp"

namespace steer_the_car
{

SteerTheCarNode::SteerTheCarNode(const rclcpp::NodeOptions & options)
:  Node("steer_the_car", options)
{
  steer_the_car_ = std::make_unique<steer_the_car::SteerTheCar>();
  const int64_t param_name = this->declare_parameter("param_name", 456);
  steer_the_car_->setParameters(param_name);
  steer_pub = this->create_publisher<autoware_auto_control_msgs::msg::AckermannControlCommand>("/control/command/control_cmd", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
  odom_sub = this->create_subscription<nav_msgs::msg::Odometry>("/localization/odometry", 1, std::bind(&SteerTheCarNode::odometry_callback, this, std::placeholders::_1));
  vel_sub = this->create_subscription<autoware_auto_vehicle_msgs::msg::VelocityReport>("/vehicle/status/velocity_status", 10, std::bind(&SteerTheCarNode::get_vel_topic, this, std::placeholders::_1));
  tf_buffer_ =
      std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ =
      std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}

void SteerTheCarNode::get_vel_topic(const autoware_auto_vehicle_msgs::msg::VelocityReport::SharedPtr msg)
{
  // std::cout <<"VELOCITY: " << msg->longitudinal_velocity << std::endl;
  longitudinal_vel_ = msg->longitudinal_velocity;
  lateral_vel_ = msg->lateral_velocity;
  heading_rate_ = msg->heading_rate;
}

void SteerTheCarNode::odometry_callback(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
{
  std::cout << msg << std::endl;
  if (first_loop)
  {
    std::ifstream file(file_path);
    if (!file.is_open()) {
        std::cout << "Failed to open the file." << std::endl;
    }

    double number1, number2;
    std::string line;

    while (std::getline(file, line)) {
        std::istringstream iss(line);
        if (!(iss >> number1 >> number2)) {
            std::cout << "Failed to read numbers from line: " << line << std::endl;
            break;
        }

        std::cout << "X: " << number1 << ", Y: " << number2 << std::endl;
        coordinates_list.push_back(std::make_tuple(number1, number2));
    }

    file.close();
    list_size = coordinates_list.size();
    first_loop = false;
  }
  else
  {
    // GETTING CURRENT CAR LOCATION
    geometry_msgs::msg::TransformStamped t;
    try {
            t = tf_buffer_->lookupTransform(
              "map", "base_link",
              tf2::TimePointZero);
          } catch (const tf2::TransformException & ex) {
            RCLCPP_INFO(this->get_logger(), "Could not transform");
          }
    cur_car_x = t.transform.translation.x;
    cur_car_y = t.transform.translation.y;
    cur_car_z = t.transform.translation.z;


    // GETTING CURRENT POINT TO FOLLOW
    if (list_index < list_size) {
        // Accessing the nth element using the indexing operator
        std::tuple<double, double> element = coordinates_list[list_index];

        // Extract the values from the tuple
        std::tie(cur_point_x, cur_point_y) = element;

        // Print the values
        std::cout << "-------------------------------------------------------------------------------------" << std::endl;
        // std::cout << "Coordinate no: " << list_index << " X: " << cur_point_x << " Y: " << cur_point_y << std::endl;
    } 
    else 
    {
        list_index = 0;
    }

    // COMPUTE LOOK-AHEAD DISTANCE
    // l_d = std::clamp(K_dd * longitudinal_vel_, min_ld, max_ld);

    // CALCULATE DISTANCE TO THE NEXT POINT
    dx = cur_car_x - cur_point_x;
    dy = cur_car_y - cur_point_y;

    distance = std::sqrt(dx * dx + dy * dy);
    std::cout << "CAR X: " << cur_car_x << "CAR Y: " << cur_car_y << std::endl;
    std::cout << "Coordinate no: " << list_index << " X: " << cur_point_x << " Y: " << cur_point_y << std::endl;
    std::cout << "DISTANCE X: " << dx << " " << "DISTANCE Y: " << dy << std::endl;
    std::cout << "Distance to point: " << distance << std::endl;
    // std::cout << "Point number: " << list_index << std::endl;

    // COMPUTE ANGLE TO NEXT POINT
    double alpha = std::atan2(dy, dx);

    // COMPUTE STEERING ANGLE
    double steering_angle = std::atan((2*wheel_base*std::sin(alpha))/l_d);
    double steering_angle_radian = -steering_angle * (M_PI / 180.0);
    steering_angle = -steering_angle/M_PI;
    // if (steering_angle > 0.5)
    // {
    //   steering_angle = 0.5;
    // }
    // if (steering_angle < -0.5)
    // {
    //   steering_angle = -0.5;
    // }

    if (distance < l_d)
    {
      list_index += 1;
    }

    // PurePursuitController controller(2.5);

    // controller.setPath(coordinates_list);

    // std::tuple<double, double> current_pose = std::make_tuple(cur_car_x, cur_car_y);

    // double steering_angle_controller = controller.calculateSteeringAngle(current_pose);
    
    std::cout << "Steering Angle: " << steering_angle << std::endl;
    std::cout << "Steering Angle Radian: " << steering_angle_radian << std::endl;
    std::cout << "Speed long: " << longitudinal_vel_ << std::endl;
    std::cout << "Speed lat: " << lateral_vel_ << std::endl;
    std::cout << "Heading lat: " << heading_rate_ << std::endl;
    std::cout << "-------------------------------------------------------------------------------------" << std::endl;
    double acc = 5.0;
    if (longitudinal_vel_ > 10.0)
    {
      acc = 0.0;
    }

    autoware_auto_control_msgs::msg::AckermannControlCommand cmd;
    cmd.stamp = this->now();
    {
      cmd.lateral.steering_tire_angle = steering_angle;
      cmd.lateral.steering_tire_rotation_rate = steering_angle_velocity;

      cmd.longitudinal.speed = 0.5;
      cmd.longitudinal.acceleration = acc;
    }
  
  prev_control_command_ = cmd;
  steer_pub->publish(cmd);

  }
}


}  // namespace steer_the_car

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(steer_the_car::SteerTheCarNode)
