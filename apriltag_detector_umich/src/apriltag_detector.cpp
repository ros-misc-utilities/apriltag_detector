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

#include <apriltag_detector/apriltag_detector.hpp>
#include <array>
#ifdef USE_CV_BRIDGE_HPP
#include <cv_bridge/cv_bridge.hpp>
#else
#include <cv_bridge/cv_bridge.h>
#endif

#include <apriltag_detector/detector_wrapper.hpp>
#include <opencv2/core/core.hpp>
#include <rclcpp_components/register_node_macro.hpp>

namespace apriltag_detector
{
ApriltagDetector::ApriltagDetector(const rclcpp::NodeOptions & options)
: Node(
    "apriltag_detector",
    rclcpp::NodeOptions(options)
      .automatically_declare_parameters_from_overrides(true))
{
  detector_.reset(new DetectorWrapper(
    get_parameter_or("tag_family", std::string("tf36h11")),
    get_parameter_or<int>("max_hamming_distance", 0)));
  detector_->setDecimateFactor(get_parameter_or("decimate_factor", 1.0));
  detector_->setQuadSigma(get_parameter_or("blur", 0.0));
  detector_->setNumberOfThreads(get_parameter_or("num_threads", 1));
  get_parameter_or(
    "image_qos_profile", imageQoSProfile_, std::string("default"));

  // publish images
  const rmw_qos_profile_t qosProf = rmw_qos_profile_default;
  imagePub_ = image_transport::create_publisher(this, "~/image_raw", qosProf);
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
  if (imagePub_.getNumSubscribers() || detectPub_->get_subscription_count()) {
    // -------------- subscribers ---------------------
    if (!isSubscribed_) {
      RCLCPP_INFO(this->get_logger(), "subscribing to images!");
      imageSub_ = image_transport::create_subscription(
        this, "image",
        std::bind(&ApriltagDetector::callback, this, std::placeholders::_1),
        "raw",
        string_to_profile(imageQoSProfile_));  // rmw_qos_profile_default);//
      isSubscribed_ = true;
    }
  } else {
    // -------------- no subscribers -------------------
    if (isSubscribed_) {
      imageSub_.shutdown();
      RCLCPP_INFO(this->get_logger(), "unsubscribing from images!");
      isSubscribed_ = false;
    }
  }
}

void ApriltagDetector::callback(
  const sensor_msgs::msg::Image::ConstSharedPtr & msg)
{
  apriltag_msgs::msg::AprilTagDetectionArray arrayMsg;
  cv_bridge::CvImageConstPtr cvImg;
  if (
    detectPub_->get_subscription_count() != 0 ||
    imagePub_.getNumSubscribers() != 0) {
    cvImg = cv_bridge::toCvShare(msg, "mono8");
    detector_->detect(cvImg->image, &arrayMsg);
    if (!cvImg) {
      RCLCPP_WARN_STREAM(
        this->get_logger(),
        "cannot convert image from " << msg->encoding << " to mono8");
      return;
    }
  }

  if (detectPub_->get_subscription_count() != 0) {
    arrayMsg.header = msg->header;
    detectPub_->publish(arrayMsg);
  }

  if (imagePub_.getNumSubscribers() != 0) {
    cv::Mat colorImg = DetectorWrapper::draw(cvImg->image, arrayMsg);
    cv_bridge::CvImage pubImg(
      msg->header, sensor_msgs::image_encodings::BGR8, colorImg);
    imagePub_.publish(pubImg.toImageMsg());
  }
}

}  // namespace apriltag_detector

RCLCPP_COMPONENTS_REGISTER_NODE(apriltag_detector::ApriltagDetector)
