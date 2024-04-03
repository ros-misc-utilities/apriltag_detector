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

#ifndef APRILTAG_DETECTOR__APRILTAG_DETECTOR_HPP_
#define APRILTAG_DETECTOR__APRILTAG_DETECTOR_HPP_

#include <apriltag_msgs/msg/april_tag_detection_array.hpp>
#include <image_transport/image_transport.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <string>

namespace apriltag_detector
{
class DetectorWrapper;  // forward decl

class ApriltagDetector : public rclcpp::Node
{
public:
  explicit ApriltagDetector(const rclcpp::NodeOptions & options);
  ~ApriltagDetector();

private:
  using ApriltagArray = apriltag_msgs::msg::AprilTagDetectionArray;
  void subscriptionCheckTimerExpired();
  void callback(const sensor_msgs::msg::Image::ConstSharedPtr & msg);
  // ------------------------  variables ------------------------------
  rclcpp::TimerBase::SharedPtr subscriptionCheckTimer_;
  rclcpp::Publisher<ApriltagArray>::SharedPtr detectPub_;
  image_transport::Subscriber imageSub_;
  image_transport::Publisher imagePub_;
  bool isSubscribed_{false};
  std::string imageQoSProfile_{"default"};
  std::shared_ptr<DetectorWrapper> detector_;
};
}  // namespace apriltag_detector
#endif  // APRILTAG_DETECTOR__APRILTAG_DETECTOR_HPP_
