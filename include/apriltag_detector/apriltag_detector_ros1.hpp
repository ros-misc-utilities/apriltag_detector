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

#ifndef APRILTAG_DETECTOR__APRILTAG_DETECTOR_ROS1_HPP_
#define APRILTAG_DETECTOR__APRILTAG_DETECTOR_ROS1_HPP_

#include <apriltag_detector_msgs/ApriltagArrayStamped.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>

#include <memory>
#include <string>

namespace apriltag_detector
{
class DetectorWrapper;  // forward decl
class ApriltagDetector
{
public:
  explicit ApriltagDetector(ros::NodeHandle & nh);
  ~ApriltagDetector() = default;

private:
  using ApriltagArray = apriltag_detector_msgs::ApriltagArrayStamped;
  void imageConnectCallback(const image_transport::SingleSubscriberPublisher &);
  void callback(const sensor_msgs::Image::ConstPtr & msg);
  // ------------------------  variables ------------------------------
  ros::NodeHandle nh_;
  ros::Publisher detectPub_;
  image_transport::Subscriber imageSub_;
  image_transport::Publisher imagePub_;
  bool isSubscribed_{false};
  std::shared_ptr<DetectorWrapper> detector_;
};
}  // namespace apriltag_detector
#endif  // APRILTAG_DETECTOR__APRILTAG_DETECTOR_ROS1_HPP_
