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

#include <cv_bridge/cv_bridge.h>

#include <apriltag_detector/apriltag_detector_ros1.hpp>
#include <apriltag_detector/detector_wrapper.hpp>
#include <array>
#include <opencv2/core/core.hpp>

namespace apriltag_detector
{
ApriltagDetector::ApriltagDetector(ros::NodeHandle & nh) : nh_(nh)

{
  detector_.reset(new DetectorWrapper(
    nh_.param<std::string>("tag_family", "tf36h11"),
    nh_.param<int>("max_hamming_distance", 0)));
  detector_->setDecimateFactor(nh_.param<double>("decimate_factor", 1.0));
  detector_->setQuadSigma(nh_.param<double>("blur_sigma", 0.0));
  detector_->setNumberOfThreads(nh_.param<int>("num_threads", 1));

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

void ApriltagDetector::callback(const sensor_msgs::Image::ConstPtr & msg)
{
  ApriltagArray arrayMsg;
  cv_bridge::CvImageConstPtr cvImg;
  if (
    detectPub_.getNumSubscribers() != 0 || imagePub_.getNumSubscribers() != 0) {
    cvImg = cv_bridge::toCvShare(msg, "mono8");
    detector_->detect(cvImg->image, &arrayMsg);
    if (!cvImg) {
      ROS_WARN_STREAM(
        "cannot convert image from " << msg->encoding << " to mono8");
      return;
    }
  }

  if (detectPub_.getNumSubscribers() != 0) {
    arrayMsg.header = msg->header;
    detectPub_.publish(arrayMsg);
  }

  if (imagePub_.getNumSubscribers() != 0) {
    cv::Mat colorImg = DetectorWrapper::draw(cvImg->image, arrayMsg);
    cv_bridge::CvImage pubImg(
      msg->header, sensor_msgs::image_encodings::BGR8, colorImg);
    imagePub_.publish(pubImg.toImageMsg());
  }
}

}  // namespace apriltag_detector
