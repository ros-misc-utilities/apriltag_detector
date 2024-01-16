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

#include "apriltag_detector/apriltag_detector_ros2.hpp"

#include <apriltag/apriltag.h>
#include <cv_bridge/cv_bridge.h>

#include <array>
#include <opencv2/core/core.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include "apriltag_detector/common.hpp"

namespace apriltag_detector_ros
{
ApriltagDetector::ApriltagDetector(const rclcpp::NodeOptions & options)
: Node(
    "apriltag_detector",
    rclcpp::NodeOptions(options)
      .automatically_declare_parameters_from_overrides(true))
{
  this->get_parameter_or("tag_family", familyName_, std::string("tf36h11"));
  this->get_parameter_or("max_hamming_distance", maxHamming_, 0);
  this->get_parameter_or("decimate_factor", decimateFactor_, 1.0);
  this->get_parameter_or("blur", blurSigma_, 0.0);
  this->get_parameter_or("num_threads", numThreads_, 1);
  this->get_parameter_or(
    "image_qos_profile", imageQoSProfile_, std::string("sensor_data"));

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
  family_ = common::make_tag_family(familyName_);
  if (!family_) {
    RCLCPP_ERROR_STREAM(
      this->get_logger(), "unknown tag family " << familyName_);
    throw std::runtime_error("bad tag family");
  }
}

ApriltagDetector::~ApriltagDetector()
{
  if (subscriptionCheckTimer_) {
    subscriptionCheckTimer_->cancel();
  }
  if (family_) {
    common::destroy_tag_family(familyName_, family_);
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

void ApriltagDetector::publishDetections(
  const zarray_t * detections, const std_msgs::msg::Header & header)
{
  ApriltagArray arrayMsg;
  arrayMsg.header = header;
  const auto numDetections = zarray_size(detections);
  for (int i = 0; i < numDetections; ++i) {
    apriltag_detection_t * t;
    zarray_get(detections, i, &t);
    apriltag_detector_msgs::msg::Apriltag tag;
    tag.id = t->id;
    tag.bits = family_->nbits;
    tag.hamming = t->hamming;
    tag.family = familyName_;
    tag.border = 1;
    tag.center.x = t->c[0];
    tag.center.y = t->c[1];
    tag.decision_margin = t->decision_margin;
    for (int j = 0; j < 4; ++j) {
      tag.corners[j].x = t->p[j][0];
      tag.corners[j].y = t->p[j][1];
    }
    arrayMsg.apriltags.push_back(tag);
  }
  detectPub_->publish(arrayMsg);
}

static void draw_line(
  cv::Mat & img, double x0, double y0, double x1, double y1,
  const cv::Scalar & col, int lw)
{
  cv::line(
    img, {static_cast<int>(x0), static_cast<int>(y0)},
    {static_cast<int>(x1), static_cast<int>(y1)}, col, lw);
}
static void draw_tag(
  cv::Mat & img, int id, const std::array<double, 2> & a,
  const std::array<std::array<double, 2>, 4> & c)
{
  (void)img;
  int lw = 2;                                   // line width
  const auto X_COLOR = CV_RGB(255, 0, 0);       // red
  const auto Y_COLOR = CV_RGB(0, 255, 0);       // green
  const auto Z_COLOR = CV_RGB(0, 0, 255);       // blue
  const auto TEXT_COLOR = CV_RGB(255, 0, 255);  // purple
  draw_line(img, c[0][0], c[0][1], c[1][0], c[1][1], X_COLOR, lw);
  draw_line(img, c[0][0], c[0][1], c[3][0], c[3][1], Y_COLOR, lw);
  draw_line(img, c[2][0], c[2][1], c[3][0], c[3][1], Z_COLOR, lw);
  draw_line(img, c[1][0], c[1][1], c[2][0], c[2][1], Z_COLOR, lw);
  const auto lt = cv::LINE_AA;  // anti-aliased line

  // write id into center of tag
  const int x_off = 5;
  const int y_off = 5;
  cv::putText(
    img, std::to_string(id), cv::Point2f(a[0] - x_off, a[1] + y_off),
    cv::FONT_HERSHEY_SIMPLEX, 1, TEXT_COLOR, 2, lt);

  // draw corner circles
  const cv::Scalar cc[4] = {X_COLOR, Y_COLOR, Z_COLOR, TEXT_COLOR};
  const int R = 2;  // radius of corner circles
  for (int i = 0; i < 4; i++) {
    cv::circle(img, cv::Point2i(c[i][0], c[i][1]), R, cc[i], 1, lt);
  }
}

void ApriltagDetector::publishImage(
  const zarray_t * detections, const std_msgs::msg::Header & header,
  const cv::Mat & img)
{
  cv::Mat colorImg;
  cv::cvtColor(img, colorImg, CV_GRAY2BGR);
  const auto numDetections = zarray_size(detections);
  for (int i = 0; i < numDetections; ++i) {
    apriltag_detection_t * t;
    zarray_get(detections, i, &t);
    draw_tag(
      colorImg, t->id, {t->c[0], t->c[1]},
      *reinterpret_cast<const std::array<std::array<double, 2>, 4> *>(
        &(t->p[0][0])));
  }

  cv_bridge::CvImage pubImg(
    header, sensor_msgs::image_encodings::BGR8, colorImg);
  imagePub_.publish(pubImg.toImageMsg());
}

void ApriltagDetector::callback(
  const sensor_msgs::msg::Image::ConstSharedPtr & msg)
{
  apriltag_detector_t * td = apriltag_detector_create();
  apriltag_detector_add_family_bits(td, family_, maxHamming_);
  td->quad_decimate = decimateFactor_;
  td->quad_sigma = blurSigma_;
  td->nthreads = numThreads_;
  auto cvImg = cv_bridge::toCvShare(msg, "mono8");
  if (!cvImg) {
    RCLCPP_WARN_STREAM(
      this->get_logger(),
      "cannot convert image from " << msg->encoding << " to mono8");
    return;
  }
  image_u8_t img{
    cvImg->image.cols, cvImg->image.rows,
    static_cast<int32_t>(cvImg->image.step), cvImg->image.data};
  zarray_t * detections = apriltag_detector_detect(td, &img);
  if (detectPub_->get_subscription_count() != 0) {
    publishDetections(detections, msg->header);
  }
  if (imagePub_.getNumSubscribers() != 0) {
    publishImage(detections, msg->header, cvImg->image);
  }
}

}  // namespace apriltag_detector_ros

RCLCPP_COMPONENTS_REGISTER_NODE(apriltag_detector_ros::ApriltagDetector)
