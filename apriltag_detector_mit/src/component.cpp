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

#include <apriltag_detector_mit/component.hpp>
#include <image_transport/image_transport.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#ifdef USE_CV_BRIDGE_HPP
#include <cv_bridge/cv_bridge.hpp>
#else
#include <cv_bridge/cv_bridge.h>
#endif

namespace apriltag_detector_mit
{
Component::Component(const rclcpp::NodeOptions & options)
: Node(
    "apriltag_detector_mit",
    rclcpp::NodeOptions(options)
      .automatically_declare_parameters_from_overrides(true)),
  detector_(new Detector())
{
  detector_->setFamily(get_parameter_or("tag_family", std::string("tf36h11")));
  in_transport_ = get_parameter_or("image_transport", std::string("raw"));
  detector_->setBlackBorder(get_parameter_or("black_border_width", 1));
  get_parameter_or(
    "image_qos_profile", image_qos_profile_, std::string("default"));

  detect_pub_ = create_publisher<ApriltagArray>("tags", 100);

  // Since the ROS2 image transport does not call back when subscribers come and go
  // must check by polling
  subscription_check_timer_ = rclcpp::create_timer(
    this, get_clock(), rclcpp::Duration(1, 0),
    std::bind(&Component::subscriptionCheckTimerExpired, this));
}

Component::~Component()
{
  if (subscription_check_timer_) {
    subscription_check_timer_->cancel();
  }
}

rmw_qos_profile_t string_to_profile(const std::string & s)
{
  if (s == "sensor_data") {
    return (rmw_qos_profile_sensor_data);
  }
  return (rmw_qos_profile_default);
}

void Component::subscriptionCheckTimerExpired()
{
  if (detect_pub_->get_subscription_count()) {
    // -------------- subscribers ---------------------
    if (!is_subscribed_) {
      RCLCPP_INFO(this->get_logger(), "subscribing to images!");
      image_sub_ = std::make_shared<image_transport::Subscriber>(
        image_transport::create_subscription(
          this, "image",
          std::bind(&Component::callback, this, std::placeholders::_1),
          in_transport_,
          string_to_profile(
            image_qos_profile_)));  // rmw_qos_profile_default);//
      is_subscribed_ = true;
    }
  } else {
    // -------------- no subscribers -------------------
    if (is_subscribed_) {
      image_sub_->shutdown();
      RCLCPP_INFO(this->get_logger(), "unsubscribing from images!");
      is_subscribed_ = false;
    }
  }
}

void Component::callback(const sensor_msgs::msg::Image::ConstSharedPtr & msg)
{
  if (detect_pub_->get_subscription_count() != 0) {
    cv_bridge::CvImageConstPtr cvImg = cv_bridge::toCvShare(msg, "mono8");
    if (!cvImg) {
      RCLCPP_WARN(get_logger(), "cannot convert image to mono!");
      return;
    }
    auto array_msg =
      std::make_unique<apriltag_msgs::msg::AprilTagDetectionArray>();
    detector_->detect(cvImg->image, array_msg.get());
    array_msg->header = msg->header;
    detect_pub_->publish(std::move(array_msg));
  }
}
}  // namespace apriltag_detector_mit

RCLCPP_COMPONENTS_REGISTER_NODE(apriltag_detector_mit::Component)
