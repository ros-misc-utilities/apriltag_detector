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

#include "apriltag_detector/apriltag_detector_ros1.hpp"

#include <apriltag/apriltag.h>
#include <cv_bridge/cv_bridge.h>

#include <array>
#include <opencv2/core/core.hpp>

#include "apriltag_detector/common.hpp"

namespace apriltag_detector_ros
{
ApriltagDetector::ApriltagDetector(ros::NodeHandle & nh) : nh_(nh)
{
  familyName_ = nh_.param<std::string>("tag_family", "tf36h11");
  maxHamming_ = nh_.param<int>("max_hamming_distance", 0);
  decimateFactor_ = nh_.param<double>("decimate_factor", 1.0);
  blurSigma_ = nh_.param<double>("blur_sigma", 0.0);
  numThreads_ = nh_.param<int>("num_threads", 1);

  // publish images
  image_transport::ImageTransport it(nh_);
  imagePub_ = it.advertise(
    "image_raw", 1,
    boost::bind(
      &ApriltagDetector::imageConnectCallback, this, boost::placeholders::_1),
    boost::bind(
      &ApriltagDetector::imageConnectCallback, this, boost::placeholders::_1));

  // publish detections
  detectPub_ = nh_.advertise<ApriltagArray>("tags", 100);

  family_ = common::make_tag_family(familyName_);
  if (!family_) {
    ROS_ERROR_STREAM("unknown tag family " << familyName_);
    throw std::runtime_error("bad tag family");
  }
}

ApriltagDetector::~ApriltagDetector()
{
  if (family_) {
    common::destroy_tag_family(familyName_, family_);
  }
}

void ApriltagDetector::imageConnectCallback(
  const image_transport::SingleSubscriberPublisher &)
{
  if (imagePub_.getNumSubscribers() || detectPub_.getNumSubscribers()) {
    // -------------- subscribers ---------------------
    if (!isSubscribed_) {
      ROS_INFO("subscribing to images!");
      image_transport::ImageTransport it(nh_);
      imageSub_ = it.subscribe("image", 2, &ApriltagDetector::callback, this);
      isSubscribed_ = true;
    }
  } else {
    // -------------- no subscribers -------------------
    if (isSubscribed_) {
      imageSub_.shutdown();
      ROS_INFO("unsubscribing from images!");
      isSubscribed_ = false;
    }
  }
}

void ApriltagDetector::publishDetections(
  const zarray_t * detections, const std_msgs::Header & header)
{
  ApriltagArray arrayMsg =
    common::detections_to_msg<ApriltagArray, apriltag_detector_msgs::Apriltag>(
      detections, family_, familyName_);
  arrayMsg.header = header;
  detectPub_.publish(arrayMsg);
}

void ApriltagDetector::publishImage(
  const zarray_t * detections, const std_msgs::Header & header,
  const cv::Mat & img)
{
  cv::Mat colorImg = common::make_image(img, detections);

  cv_bridge::CvImage pubImg(
    header, sensor_msgs::image_encodings::BGR8, colorImg);
  imagePub_.publish(pubImg.toImageMsg());
}

void ApriltagDetector::callback(const sensor_msgs::Image::ConstPtr & msg)
{
  apriltag_detector_t * td = apriltag_detector_create();
  apriltag_detector_add_family_bits(td, family_, maxHamming_);
  td->quad_decimate = decimateFactor_;
  td->quad_sigma = blurSigma_;
  td->nthreads = numThreads_;
  auto cvImg = cv_bridge::toCvShare(msg, "mono8");
  if (!cvImg) {
    ROS_WARN_STREAM(
      "cannot convert image from " << msg->encoding << " to mono8");
    return;
  }
  image_u8_t img{
    cvImg->image.cols, cvImg->image.rows,
    static_cast<int32_t>(cvImg->image.step), cvImg->image.data};
  // this runs the actual apriltag detector!
  zarray_t * detections = apriltag_detector_detect(td, &img);
  if (detectPub_.getNumSubscribers() != 0) {
    publishDetections(detections, msg->header);
  }
  if (imagePub_.getNumSubscribers() != 0) {
    publishImage(detections, msg->header, cvImg->image);
  }
}

}  // namespace apriltag_detector_ros
