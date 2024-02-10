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

#ifndef APRILTAG_DETECTOR__DETECTOR_WRAPPER_BASE_HPP_
#define APRILTAG_DETECTOR__DETECTOR_WRAPPER_BASE_HPP_

#include <memory>
#include <string>

namespace cv
{
struct Mat;  // forward decl
}

namespace apriltag_detector_ros
{
class DetectorWrapperBase
{
public:
  DetectorWrapperBase(const std::string & fam, int hamming);
  ~DetectorWrapperBase();

  void setQuadSigma(double blur);
  void setDecimateFactor(double decimate_factor);
  void setNumberOfThreads(int num_threads);

protected:
  void * detectTags(const cv::Mat & img);
  void freeDetections(void * det);
  // ------------------------  variables ------------------------------
  void * detector_{nullptr};
  void * family_{nullptr};
  std::string family_name_;
};
}  // namespace apriltag_detector_ros
#endif  // APRILTAG_DETECTOR__DETECTOR_WRAPPER_BASE_HPP_
