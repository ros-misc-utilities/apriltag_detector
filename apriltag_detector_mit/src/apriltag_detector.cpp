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

static void detectAndConvert(
  rclcpp::Node * node, const std::string & family,
  const std::shared_ptr<AprilTags::TagDetector> & detector,
  const sensor_msgs::msg::Image::ConstSharedPtr & img, ApriltagArray * arrayMsg)
{
  // Detection
  cv_bridge::CvImageConstPtr cvImg = cv_bridge::toCvShare(img, "mono8");
  if (!cvImg) {
    RCLCPP_WARN_STREAM(node->get_logger(), "cannot convert image to mono!");
    return;
  }
  auto detections = detector->ExtractTags(cvImg->image);

  // Convert to  Apriltag message
  arrayMsg->detections.resize(detections.size());

  for (size_t i = 0; i < detections.size(); i++) {
    const AprilTags::TagDetection & td = detections[i];
    auto & apriltag = arrayMsg->detections[i];
    apriltag.family = family;
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
  /*
  detector_->setDecimateFactor(get_parameter_or("decimate_factor", 1.0));
  detector_->setQuadSigma(get_parameter_or("blur", 0.0));
  detector_->setNumberOfThreads(get_parameter_or("num_threads", 1));
  */
  detector_->set_black_border(get_parameter_or("border_bits", 1));
  get_parameter_or(
    "image_qos_profile", imageQoSProfile_, std::string("default"));

  // publish detections
  detectPub_ = create_publisher<ApriltagArray>("~/tags", 100);

  // Since the ROS2 image transport does not call back when subscribers come and go
  // must check by polling
  subscriptionCheckTimer_ = rclcpp::create_timer(
    this, get_clock(), rclcpp::Duration(1, 0),
    std::bind(&ApriltagDetector::subscriptionCheckTimerExpired, this));
}

ApriltagDetector::~ApriltagDetector()
{
  if (subscriptionCheckTimer_) {
    subscriptionCheckTimer_->cancel();
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
  if (detectPub_->get_subscription_count()) {
    // -------------- subscribers ---------------------
    if (!isSubscribed_) {
      RCLCPP_INFO(this->get_logger(), "subscribing to images!");
      imageSub_ = std::make_shared<image_transport::Subscriber>(
        image_transport::create_subscription(
          this, "image",
          std::bind(&ApriltagDetector::callback, this, std::placeholders::_1),
          in_transport_,
          string_to_profile(imageQoSProfile_)));  // rmw_qos_profile_default);//
      isSubscribed_ = true;
    }
  } else {
    // -------------- no subscribers -------------------
    if (isSubscribed_) {
      imageSub_->shutdown();
      RCLCPP_INFO(this->get_logger(), "unsubscribing from images!");
      isSubscribed_ = false;
    }
  }
}

void ApriltagDetector::callback(
  const sensor_msgs::msg::Image::ConstSharedPtr & msg)
{
  apriltag_msgs::msg::AprilTagDetectionArray arrayMsg;
  if (detectPub_->get_subscription_count() != 0) {
    detectAndConvert(this, family_, detector_, msg, &arrayMsg);
    arrayMsg.header = msg->header;
    detectPub_->publish(arrayMsg);
  }
}

}  // namespace apriltag_detector_mit

RCLCPP_COMPONENTS_REGISTER_NODE(apriltag_detector_mit::ApriltagDetector)
