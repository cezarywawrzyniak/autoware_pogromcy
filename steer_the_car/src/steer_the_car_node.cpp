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

//                            kp    ki   kd   
PIDController speedController(10.0, 0.1, 0.2, 3.0, 2.0);

namespace steer_the_car
{

SteerTheCarNode::SteerTheCarNode(const rclcpp::NodeOptions & options)
:  Node("steer_the_car", options)
{
  steer_the_car_ = std::make_unique<steer_the_car::SteerTheCar>();
  const int64_t param_name = this->declare_parameter("param_name", 456);
  steer_the_car_->setParameters(param_name);
  steer_pub = this->create_publisher<autoware_auto_control_msgs::msg::AckermannControlCommand>("/control/command/control_cmd", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
  vel_sub = this->create_subscription<autoware_auto_vehicle_msgs::msg::VelocityReport>("/vehicle/status/velocity_status", 10, std::bind(&SteerTheCarNode::get_vel_topic, this, std::placeholders::_1));
  pub_marker = this->create_publisher<visualization_msgs::msg::Marker>("/marker", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
  pub_marker_list = this->create_publisher<visualization_msgs::msg::Marker>("/marker_list", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
  tf_buffer_ =
      std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ =
      std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}


void SteerTheCarNode::pub_arrow()
{
  marker.header.frame_id = "map"; 
  marker.id = 0;
  marker.type = visualization_msgs::msg::Marker::SPHERE;
  marker.action = 0;
  marker.lifetime = rclcpp::Duration(0, 1e9); 

  marker.scale.x = 0.2;
  marker.scale.y = 0.2;
  marker.scale.z = 0.2;
  marker.color.a = 1.0;
  marker.color.r = 0.0;
  marker.color.g = 1.0;
  marker.color.b = 0.0;
  pub_marker->publish(marker);
}

void SteerTheCarNode::pub_trajectory()
{
  marker2.header.frame_id = "map";
  marker2.id = 0;
  marker2.type = 7;
  marker2.action = 0;
  marker2.pose = geometry_msgs::msg::Pose();
  marker2.color.a = 1.0;
  marker2.color.b = 1.0;
  marker2.color.g = 1.0;
  marker2.color.r = 1.0;
  marker2.scale.x = 0.2;
  marker2.scale.y = 0.2;
  marker2.scale.z = 0.2;
  pub_marker_list->publish(marker2);
}

void SteerTheCarNode::get_vel_topic(const autoware_auto_vehicle_msgs::msg::VelocityReport::SharedPtr msg)
{
  longitudinal_vel_ = msg->longitudinal_velocity;
  lateral_vel_ = msg->lateral_velocity;
  heading_rate_ = msg->heading_rate;

  if (first_loop)
  {
    std::ifstream file(file_path);
    if (!file.is_open()) {
        std::cout << "Failed to open the file." << std::endl;
    }

    double number1, number2, number3;
    std::string line;

    while (std::getline(file, line)) {
        std::istringstream iss(line);
        if (!(iss >> number1 >> number2 >> number3)) {
            std::cout << "Failed to read numbers from line: " << line << std::endl;
            break;
        }

        std::cout << "X: " << number1 << ", Y: " << number2 << ", SPEED: " << number3 << std::endl;
        coordinates_list.push_back(std::make_tuple(number1, number2, number3));

        auto point = geometry_msgs::msg::Point();
        point.x = number1;
        point.y = number2;
        marker2.points.push_back(point);
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

    marker.pose.position.x = cur_point_x;
    marker.pose.position.y = cur_point_y;
    marker.pose.position.z = cur_car_z;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    pub_arrow();
    pub_trajectory();

    // GETTING CURRENT POINT TO FOLLOW
    if (list_index < list_size) {
        // Accessing the nth element using the indexing operator
        std::tuple<double, double, double> element = coordinates_list[list_index];

        // Extract the values from the tuple
        std::tie(cur_point_x, cur_point_y, cur_target_speed) = element;        
    } 
    else 
    {
        list_index = 0;
    }

    tf2::Quaternion quaternion;
    tf2::fromMsg(t.transform.rotation, quaternion);

    // Convert the quaternion to euler angles (roll, pitch, yaw)
    double roll, pitch, yaw;
    tf2::Matrix3x3(quaternion).getRPY(roll, pitch, yaw);

    // Convert the yaw from radians to degrees
    double yaw_degrees = yaw * 180.0 / M_PI;

    // Print the yaw angle
    std::cout << "-------------------------------------------------------------------------------------" << std::endl;
    std::cout << "Yaw degrees: " << yaw_degrees << std::endl;

    // COMPUTE LOOK-AHEAD DISTANCE
    l_d = std::clamp(K_dd * longitudinal_vel_, min_ld, max_ld);
    std::cout << "Look ahead distance: " << l_d << std::endl;

    // CALCULATE DISTANCE TO THE NEXT POINT
    dx = cur_car_x - cur_point_x;
    dy = cur_car_y - cur_point_y;

    distance = std::sqrt((dx * dx) + (dy * dy));
    std::cout << "Car X: " << cur_car_x << " " << "Car Y: " << cur_car_y << std::endl;
    std::cout << "Coordinate no: " << list_index << " X: " << cur_point_x << " Y: " << cur_point_y << " Speed: " << cur_target_speed << std::endl;
    std::cout << "Distance to point: " << distance << std::endl;
    std::cout << "Point number: " << list_index << std::endl;

    // COMPUTE ANGLE TO NEXT POINT
    double alpha = std::atan2(dy, dx);

    // COMPUTE STEERING ANGLE
    double steering_angle = std::atan((2*wheel_base*std::sin(alpha - yaw))/l_d);
    steering_angle = -steering_angle;
    steering_angle = std::max(-0.5, std::min(steering_angle, 0.5));

    if (distance < l_d)
    {
      list_index += 1;
    }
    
    std::cout << "Steering Angle: " << steering_angle << std::endl;
    std::cout << "Speed: " << longitudinal_vel_ << std::endl;

    speedController.setDesiredSpeed(cur_target_speed);
    calculated_acc = speedController.calculateAcceleration(longitudinal_vel_);
    std::cout << "Acceleration: " << calculated_acc << std::endl;
    std::cout << "-------------------------------------------------------------------------------------" << std::endl;

  
    // double acc = 1.0;
    // if (longitudinal_vel_ > 3.0)
    // {
    //   acc = 0.0;
    // }

    autoware_auto_control_msgs::msg::AckermannControlCommand cmd;
    cmd.stamp = this->now();
    {
      cmd.lateral.steering_tire_angle = steering_angle;
      cmd.lateral.steering_tire_rotation_rate = steering_angle_velocity;

      cmd.longitudinal.speed = 0.5;
      cmd.longitudinal.acceleration = calculated_acc;
    }
  
  prev_control_command_ = cmd;
  steer_pub->publish(cmd);

  }
}


}  // namespace steer_the_car

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(steer_the_car::SteerTheCarNode)
