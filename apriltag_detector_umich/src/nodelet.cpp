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

#include <nodelet/nodelet.h>
#include <ros/ros.h>

#include <memory>

#include "apriltag_detector/apriltag_detector_ros1.hpp"

namespace apriltag_detector
{
class ApriltagDetectorNodelet : public nodelet::Nodelet
{
public:
  void onInit() override
  {
    nh_ = getPrivateNodeHandle();
    node_ = std::make_shared<ApriltagDetector>(nh_);
  }

private:
  // ------ variables --------
  std::shared_ptr<ApriltagDetector> node_;
  ros::NodeHandle nh_;
};
}  // namespace apriltag_detector

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(
  apriltag_detector::ApriltagDetectorNodelet, nodelet::Nodelet)
