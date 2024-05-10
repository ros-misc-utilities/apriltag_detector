// -*-c++-*---------------------------------------------------------------------------------------
// Copyright 2024 Bernd Pfrommer <bernd.pfrommer@gmail.com>
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef APRILTAG_DETECTOR_UMICH__COMPONENT_HPP_
#define APRILTAG_DETECTOR_UMICH__COMPONENT_HPP_

#include <apriltag_detector_umich/detector.hpp>
#include <apriltag_msgs/msg/april_tag_detection_array.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <string>

namespace image_transport
{
class Subscriber;  // forward decl
}

namespace apriltag_detector_umich
{
class Detector;  // forward decl

class Component : public rclcpp::Node
{
public:
  using ApriltagArray = apriltag_msgs::msg::AprilTagDetectionArray;
  using Image = sensor_msgs::msg::Image;
  explicit Component(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~Component();

private:
  void subscriptionCheckTimerExpired();
  void callback(const sensor_msgs::msg::Image::ConstSharedPtr & msg);
  // ------------------------  variables ------------------------------
  rclcpp::TimerBase::SharedPtr subscription_check_timer_;
  rclcpp::Publisher<ApriltagArray>::SharedPtr detect_pub_;
  std::shared_ptr<image_transport::Subscriber> image_sub_;
  bool is_subscribed_{false};
  std::string image_qos_profile_{"default"};
  std::string in_transport_{"raw"};
  std::shared_ptr<Detector> detector_;
};
}  // namespace apriltag_detector_umich
#endif  // APRILTAG_DETECTOR_UMICH__COMPONENT_HPP_
