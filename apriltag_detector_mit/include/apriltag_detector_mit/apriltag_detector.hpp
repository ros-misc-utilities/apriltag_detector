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

#ifndef APRILTAG_DETECTOR_MIT__APRILTAG_DETECTOR_HPP_
#define APRILTAG_DETECTOR_MIT__APRILTAG_DETECTOR_HPP_

#include <apriltag_msgs/msg/april_tag_detection_array.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <string>

namespace AprilTags
{
class TagDetector;
}

namespace image_transport
{
class Subscriber;
}

namespace image_transport
{
class Subscriber;
}

namespace apriltag_detector_mit
{
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
  std::string family_;
  rclcpp::TimerBase::SharedPtr subscriptionCheckTimer_;
  rclcpp::Publisher<ApriltagArray>::SharedPtr detectPub_;
  std::shared_ptr<image_transport::Subscriber> imageSub_;
  bool isSubscribed_{false};
  std::string imageQoSProfile_{"default"};
  std::string in_transport_{"raw"};
  std::shared_ptr<AprilTags::TagDetector> detector_;
};
}  // namespace apriltag_detector_mit
#endif  // APRILTAG_DETECTOR_MIT__APRILTAG_DETECTOR_HPP_
