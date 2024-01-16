// -*-c++-*---------------------------------------------------------------------------------------
// Copyright 2022 Bernd Pfrommer <bernd.pfrommer@gmail.com>
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

#ifndef APRILTAG_DETECTOR__APRILTAG_DETECTOR_ROS2_HPP_
#define APRILTAG_DETECTOR__APRILTAG_DETECTOR_ROS2_HPP_

#include <apriltag_detector_msgs/msg/apriltag_array_stamped.hpp>
#include <image_transport/image_transport.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <string>

class apriltag_family;  // forward decl
struct zarray;
typedef struct zarray zarray_t;  // forward decl
namespace cv
{
class Mat;  // forward decl
}

namespace apriltag_detector_ros
{
class ApriltagDetector : public rclcpp::Node
{
public:
  explicit ApriltagDetector(const rclcpp::NodeOptions & options);
  ~ApriltagDetector();

private:
  using ApriltagArray = apriltag_detector_msgs::msg::ApriltagArrayStamped;
  void subscriptionCheckTimerExpired();
  void callback(const sensor_msgs::msg::Image::ConstSharedPtr & msg);
  void publishDetections(
    const zarray_t * detections, const std_msgs::msg::Header & header);
  void publishImage(
    const zarray_t * detections, const std_msgs::msg::Header & header,
    const cv::Mat & img);
  // ------------------------  variables ------------------------------
  rclcpp::TimerBase::SharedPtr subscriptionCheckTimer_;
  rclcpp::Publisher<ApriltagArray>::SharedPtr detectPub_;
  image_transport::Subscriber imageSub_;
  image_transport::Publisher imagePub_;
  bool isSubscribed_{false};
  double decimateFactor_{1};
  double blurSigma_{0};
  int numThreads_{1};
  std::string familyName_;
  apriltag_family * family_;
  int maxHamming_{1};
  std::string imageQoSProfile_{"default"};
};
}  // namespace apriltag_detector_ros
#endif  // APRILTAG_DETECTOR__APRILTAG_DETECTOR_ROS2_HPP_
