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

#include <apriltag_detector/convert_detections.hpp>
#include <apriltag_detector/detector_wrapper.hpp>
#include <apriltag_detector/draw_msg.hpp>

namespace apriltag_detector
{
DetectorWrapper::DetectorWrapper(const std::string & fam, int ham)
: DetectorWrapperBase(fam, ham)
{
}

void DetectorWrapper::detect(const cv::Mat & img, ApriltagArray * arrayMsg)
{
  void * det = this->detectTags(img);
  apriltag_detector_ros::convert_detections(det, family_name_, arrayMsg);
  this->freeDetections(det);
}

cv::Mat DetectorWrapper::draw(const cv::Mat & img, const ApriltagArray & msg)
{
  return (draw_msg(img, msg));
}

}  // namespace apriltag_detector
