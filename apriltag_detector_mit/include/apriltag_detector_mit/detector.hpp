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

#ifndef APRILTAG_DETECTOR_MIT__DETECTOR_HPP_
#define APRILTAG_DETECTOR_MIT__DETECTOR_HPP_

#include <apriltag_detector/detector.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <string>

namespace AprilTags
{
class TagDetector;
}

namespace apriltag_detector_mit
{
class Detector : public apriltag_detector::Detector
{
public:
  explicit Detector();
  ~Detector();
  void detect(const Image * img, ApriltagArray * tags) final;
  void setFamily(const std::string & fam) final;
  void setBlackBorder(int width) final;
  void setDecimateFactor(double) final {}
  void setQuadSigma(double) final {}
  void setNumberOfThreads(int) final {}
  const std::string & getFamily() const final { return (family_); }

private:
  // ------------------------  variables ------------------------------
  std::string family_{"tf36h11"};
  int black_border_bits_{1};
  std::shared_ptr<AprilTags::TagDetector> detector_;
};
}  // namespace apriltag_detector_mit
#endif  // APRILTAG_DETECTOR_MIT__DETECTOR_HPP_
