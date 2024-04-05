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

#ifndef APRILTAG_DETECTOR_UMICH__CONVERT_DETECTIONS_HPP_
#define APRILTAG_DETECTOR_UMICH__CONVERT_DETECTIONS_HPP_

#include <apriltag_msgs/msg/april_tag_detection_array.hpp>

namespace apriltag_detector_umich
{
using ApriltagArray = apriltag_msgs::msg::AprilTagDetectionArray;
using Tag = apriltag_msgs::msg::AprilTagDetection;
void convert_detections(
  void * detections, const std::string & fam, ApriltagArray * arrayMsg);
}  // namespace apriltag_detector_umich

#endif  // APRILTAG_DETECTOR_UMICH__CONVERT_DETECTIONS_HPP_
