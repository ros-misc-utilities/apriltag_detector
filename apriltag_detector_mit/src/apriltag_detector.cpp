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

#include <apriltag_mit/apriltag_mit.h>

#include <apriltag_detector_mit/apriltag_detector.hpp>
#include <image_transport/image_transport.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#ifdef USE_CV_BRIDGE_HPP
#include <cv_bridge/cv_bridge.hpp>
#else
#include <cv_bridge/cv_bridge.h>
#endif

using ApriltagArray = apriltag_msgs::msg::AprilTagDetectionArray;

static const std::unordered_map<std::string, AprilTags::TagCodes>
  family_code_map = {
    {{"tf36h11", AprilTags::tag_codes_36h11},
     {"tf25h9", AprilTags::tag_codes_25h9},
     {"tf16h5", AprilTags::tag_codes_16h5}}};

static std::shared_ptr<AprilTags::TagDetector> make_detector(
  rclcpp::Node * node, const std::string & fam)
{
  const auto it = family_code_map.find(fam);
  if (it == family_code_map.end()) {
    RCLCPP_ERROR_STREAM(node->get_logger(), "invalid tag family: " << fam);
    return (nullptr);
  }
  return (std::make_shared<AprilTags::TagDetector>(it->second));
}

namespace apriltag_detector_mit
{
ApriltagDetector::ApriltagDetector(const rclcpp::NodeOptions & options)
: Node(
    "apriltag_detector_mit",
    rclcpp::NodeOptions(options)
      .automatically_declare_parameters_from_overrides(true))
{
  family_ = get_parameter_or("tag_family", std::string("tf36h11"));
  detector_ = make_detector(this, family_);
  in_transport_ = get_parameter_or("image_transport", std::string("raw"));
  if (!detector_) {
    throw(std::runtime_error("invalid tag family specified!"));
  }
  detector_->set_black_border(get_parameter_or("border_bits", 1));
  get_parameter_or(
    "image_qos_profile", image_qos_profile_, std::string("default"));

  // publish detections
  detect_pub_ = create_publisher<ApriltagArray>("~/tags", 100);

  // Since the ROS2 image transport does not call back when subscribers come and go
  // must check by polling
  subscription_check_timer_ = rclcpp::create_timer(
    this, get_clock(), rclcpp::Duration(1, 0),
    std::bind(&ApriltagDetector::subscriptionCheckTimerExpired, this));
}

ApriltagDetector::~ApriltagDetector()
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

void ApriltagDetector::subscriptionCheckTimerExpired()
{
  if (detect_pub_->get_subscription_count()) {
    // -------------- subscribers ---------------------
    if (!is_subscribed_) {
      RCLCPP_INFO(this->get_logger(), "subscribing to images!");
      image_sub_ = std::make_shared<image_transport::Subscriber>(
        image_transport::create_subscription(
          this, "~/image",
          std::bind(&ApriltagDetector::callback, this, std::placeholders::_1),
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

void ApriltagDetector::callback(
  const sensor_msgs::msg::Image::ConstSharedPtr & msg)
{
  auto array_msg =
    std::make_unique<apriltag_msgs::msg::AprilTagDetectionArray>();
  if (detect_pub_->get_subscription_count() != 0) {
    detect(msg.get(), array_msg.get());
    array_msg->header = msg->header;
    detect_pub_->publish(std::move(array_msg));
  }
}

void ApriltagDetector::detect(const Image * img, ApriltagArray * tags)
{
  // Detection
  cv_bridge::CvImageConstPtr cvImg = cv_bridge::toCvShare(
    std::shared_ptr<const Image>(img, [](const Image *) {}), "mono8");
  if (!cvImg) {
    RCLCPP_WARN(
      get_logger(),
      "cannot convert image to mono!sensor_msgs::msg::Image::ConstSharedPtr & "
      "img,");
    return;
  }
  auto detections = detector_->ExtractTags(cvImg->image);

  // Convert to  Apriltag message
  tags->detections.resize(detections.size());

  for (size_t i = 0; i < detections.size(); i++) {
    const AprilTags::TagDetection & td = detections[i];
    auto & apriltag = tags->detections[i];
    apriltag.family = family_;
    apriltag.id = td.id;
    apriltag.hamming = td.hamming_distance;
    apriltag.goodness = 0;         // what is that?,
    apriltag.decision_margin = 0;  // not supported by MIT detector
    apriltag.centre.x = td.cxy.x;
    apriltag.centre.y = td.cxy.y;
    for (size_t c = 0; c < 4; ++c) {
      apriltag.corners[c].x = td.p[c].x;
      apriltag.corners[c].y = td.p[c].y;
    }
  }
}
}  // namespace apriltag_detector_mit

RCLCPP_COMPONENTS_REGISTER_NODE(apriltag_detector_mit::ApriltagDetector)
PLUGINLIB_EXPORT_CLASS(
  apriltag_detector_mit::ApriltagDetector, apriltag_detector::ApriltagDetector)
