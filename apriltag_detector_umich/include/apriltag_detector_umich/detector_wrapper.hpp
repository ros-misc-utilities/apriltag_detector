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

#ifndef APRILTAG_DETECTOR_UMICH__DETECTOR_WRAPPER_HPP_
#define APRILTAG_DETECTOR_UMICH__DETECTOR_WRAPPER_HPP_

#include <apriltag_detector_umich/detector_wrapper_base.hpp>
#include <apriltag_msgs/msg/april_tag_detection_array.hpp>
#include <memory>
#include <opencv2/core/core.hpp>
#include <string>

namespace apriltag_detector_umich
{
class DetectorWrapper : public apriltag_detector_umich::DetectorWrapperBase
{
public:
  using SharedPtr = std::shared_ptr<DetectorWrapper>;
  using ApriltagArray = apriltag_msgs::msg::AprilTagDetectionArray;

  DetectorWrapper(const std::string & fam, int ham);
  ~DetectorWrapper() = default;

  void detect(const cv::Mat & img, ApriltagArray * arrayMsg);
  static cv::Mat draw(const cv::Mat & img, const ApriltagArray & msg);
};
}  // namespace apriltag_detector_umich
#endif  // APRILTAG_DETECTOR_UMICH__DETECTOR_WRAPPER_HPP_
