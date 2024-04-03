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

#ifndef APRILTAG_DETECTOR__DRAW_MSG_HPP_
#define APRILTAG_DETECTOR__DRAW_MSG_HPP_

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "draw_tag.hpp"

namespace apriltag_detector
{
template <class T>
static cv::Mat draw_msg(const cv::Mat & img, const T & msg)
{
  cv::Mat colorImg;
  cv::cvtColor(img, colorImg, cv::COLOR_GRAY2BGR);
#ifdef USING_ROS_1
  const auto & all_tags = msg.apriltags;
#else
  const auto & all_tags = msg.detections;
#endif
  for (const auto & tag : all_tags) {
    const auto & c = tag.corners;
    const std::array<std::array<double, 2>, 4> corners{
      {{c[0].x, c[0].y}, {c[1].x, c[1].y}, {c[2].x, c[2].y}, {c[3].x, c[3].y}}};
#ifdef USING_ROS_1
    draw_tag::draw(colorImg, tag.id, {tag.center.x, tag.center.y}, corners);
#else
    draw_tag::draw(colorImg, tag.id, {tag.centre.x, tag.centre.y}, corners);
#endif
  }
  return (colorImg);
}

}  // namespace apriltag_detector
#endif  // APRILTAG_DETECTOR__DRAW_MSG_HPP_
