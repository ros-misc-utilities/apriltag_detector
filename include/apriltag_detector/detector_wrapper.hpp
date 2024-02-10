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

#ifndef APRILTAG_DETECTOR__DETECTOR_WRAPPER_HPP_
#define APRILTAG_DETECTOR__DETECTOR_WRAPPER_HPP_

#include <apriltag_detector/detector_wrapper_base.hpp>
#include <memory>
#include <opencv2/core/core.hpp>
#include <string>

#ifdef USING_ROS_1
#include <apriltag_detector_msgs/ApriltagArrayStamped.h>
#else
#include <apriltag_msgs/msg/april_tag_detection_array.hpp>
#endif

namespace apriltag_detector
{
class DetectorWrapper : public apriltag_detector_ros::DetectorWrapperBase
{
public:
  using SharedPtr = std::shared_ptr<DetectorWrapper>;
#ifdef USING_ROS_1
  using ApriltagArray = apriltag_detector_msgs::ApriltagArrayStamped;
#else
  using ApriltagArray = apriltag_msgs::msg::AprilTagDetectionArray;
#endif

  DetectorWrapper(const std::string & fam, int ham);
  ~DetectorWrapper() = default;

  void detect(const cv::Mat & img, ApriltagArray * arrayMsg);
  static cv::Mat draw(const cv::Mat & img, const ApriltagArray & msg);
};
}  // namespace apriltag_detector
#endif  // APRILTAG_DETECTOR__DETECTOR_WRAPPER_HPP_
