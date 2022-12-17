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

#ifndef APRILTAG_DETECTOR__COMMON_HPP_
#define APRILTAG_DETECTOR__COMMON_HPP_

#include <apriltag/apriltag.h>

#include <string>

namespace cv
{
class Mat;  // forward decl
}

namespace apriltag_detector_ros
{
namespace common
{
apriltag_family * make_tag_family(const std::string & name);
void destroy_tag_family(const std::string & name, apriltag_family * tf);
cv::Mat make_image(const cv::Mat & img, const zarray_t * detections);

template <class T, class A>
T detections_to_msg(
  const zarray_t * detections, const apriltag_family_t * fam,
  const std::string & fname)
{
  T arrayMsg;
  const auto numDetections = zarray_size(detections);
  for (int i = 0; i < numDetections; ++i) {
    apriltag_detection_t * t;
    zarray_get(detections, i, &t);
    A tag;
    tag.id = t->id;
    tag.bits = fam->nbits;
    tag.hamming = t->hamming;
    tag.family = fname;
    tag.border = 1;
    tag.center.x = t->c[0];
    tag.center.y = t->c[1];
    tag.decision_margin = t->decision_margin;
    for (int j = 0; j < 4; ++j) {
      tag.corners[j].x = t->p[j][0];
      tag.corners[j].y = t->p[j][1];
    }
    arrayMsg.apriltags.push_back(tag);
  }
  return (arrayMsg);
}

}  // namespace common
}  // namespace apriltag_detector_ros
#endif  // APRILTAG_DETECTOR__COMMON_HPP_
