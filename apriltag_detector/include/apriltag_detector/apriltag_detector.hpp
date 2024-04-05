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

#ifndef APRILTAG_DETECTOR__APRILTAG_DETECTOR_HPP_
#define APRILTAG_DETECTOR__APRILTAG_DETECTOR_HPP_

#include <apriltag_msgs/msg/april_tag_detection_array.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace apriltag_detector
{
class ApriltagDetector
{
public:
  virtual ~ApriltagDetector() {}
  virtual void detect(
    const sensor_msgs::msg::Image * img,
    apriltag_msgs::msg::AprilTagDetectionArray * tags) = 0;
};
}  // namespace apriltag_detector
#endif  // APRILTAG_DETECTOR__APRILTAG_DETECTOR_HPP_
