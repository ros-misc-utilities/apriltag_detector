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

#include <apriltag.h>

#include <apriltag_detector/convert_detections.hpp>

namespace apriltag_detector_ros
{

void convert_detections(
  void * det, const std::string & fam, ApriltagArray * arrayMsg)
{
  const zarray_t * detections = reinterpret_cast<zarray_t *>(det);
  const auto numDetections = zarray_size(detections);
  for (int i = 0; i < numDetections; ++i) {
    apriltag_detection_t * t;
    zarray_get(detections, i, &t);
    Tag tag;
    tag.id = t->id;
    tag.hamming = t->hamming;
    tag.family = fam;
    tag.decision_margin = t->decision_margin;
    for (int j = 0; j < 4; ++j) {
      tag.corners[j].x = t->p[j][0];
      tag.corners[j].y = t->p[j][1];
    }
#ifdef USING_ROS_1
    tag.center.x = t->c[0];
    tag.center.y = t->c[1];
    arrayMsg->apriltags.push_back(tag);
#else
    tag.goodness = 0;  // what is that?,
    tag.centre.x = t->c[0];
    tag.centre.y = t->c[1];
    arrayMsg->detections.push_back(tag);
#endif
  }
}

}  // namespace apriltag_detector_ros
