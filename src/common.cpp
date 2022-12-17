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

#include "apriltag_detector/common.hpp"

#include <apriltag.h>
#include <tag16h5.h>
#include <tag25h9.h>
#include <tag36h11.h>
#include <tagCircle21h7.h>
#include <tagCircle49h12.h>
#include <tagCustom48h12.h>
#include <tagStandard41h12.h>
#include <tagStandard52h13.h>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

namespace apriltag_detector_ros
{
namespace common
{
apriltag_family * make_tag_family(const std::string & name)
{
  apriltag_family * tf{0};
  if (name == "tf36h11") {
    tf = tag36h11_create();
  } else if (name == "tf25h9") {
    tf = tag25h9_create();
  } else if (name == "tf16h5") {
    tf = tag16h5_create();
  } else if (name == "tfCircle21h7") {
    tf = tagCircle21h7_create();
  } else if (name == "tfCircle49h12") {
    tf = tagCircle49h12_create();
  } else if (name == "tfStandard41h12") {
    tf = tagStandard41h12_create();
  } else if (name == "tfStandard52h13") {
    tf = tagStandard52h13_create();
  } else if (name == "tfCustom48h12") {
    tf = tagCustom48h12_create();
  } else {
    tf = NULL;
  }
  return (tf);
}

void destroy_tag_family(const std::string & name, apriltag_family * tf)
{
  if (name == "tf36h11") {
    tag36h11_destroy(tf);
  } else if (name == "tf25h9") {
    tag25h9_destroy(tf);
  } else if (name == "tf16h5") {
    tag16h5_destroy(tf);
  } else if (name == "tfCircle21h7") {
    tagCircle21h7_destroy(tf);
  } else if (name == "tfCircle49h12") {
    tagCircle49h12_destroy(tf);
  } else if (name == "tfStandard41h12") {
    tagStandard41h12_destroy(tf);
  } else if (name == "tfStandard52h13") {
    tagStandard52h13_destroy(tf);
  } else if (name == "tfCustom48h12") {
    tagCustom48h12_destroy(tf);
  }
}

static void draw_line(
  cv::Mat & img, double x0, double y0, double x1, double y1,
  const cv::Scalar & col, int lw)
{
  cv::line(
    img, {static_cast<int>(x0), static_cast<int>(y0)},
    {static_cast<int>(x1), static_cast<int>(y1)}, col, lw);
}

static void draw_tag(
  cv::Mat & img, int id, const std::array<double, 2> & a,
  const std::array<std::array<double, 2>, 4> & c)
{
  (void)img;
  int lw = 2;                                   // line width
  const auto X_COLOR = cv::Vec3b(255, 0, 0);    // red
  const auto Y_COLOR = CV_RGB(0, 255, 0);       // green
  const auto Z_COLOR = CV_RGB(0, 0, 255);       // blue
  const auto TEXT_COLOR = CV_RGB(255, 0, 255);  // purple
  draw_line(img, c[0][0], c[0][1], c[1][0], c[1][1], X_COLOR, lw);
  draw_line(img, c[0][0], c[0][1], c[3][0], c[3][1], Y_COLOR, lw);
  draw_line(img, c[2][0], c[2][1], c[3][0], c[3][1], Z_COLOR, lw);
  draw_line(img, c[1][0], c[1][1], c[2][0], c[2][1], Z_COLOR, lw);
  const auto lt = cv::LINE_AA;  // anti-aliased line

  // write id into center of tag
  const int x_off = 5;
  const int y_off = 5;
  cv::putText(
    img, std::to_string(id), cv::Point2f(a[0] - x_off, a[1] + y_off),
    cv::FONT_HERSHEY_SIMPLEX, 1, TEXT_COLOR, 2, lt);

  // draw corner circles
  const cv::Scalar cc[4] = {X_COLOR, Y_COLOR, Z_COLOR, TEXT_COLOR};
  const int R = 2;  // radius of corner circles
  for (int i = 0; i < 4; i++) {
    cv::circle(img, cv::Point2i(c[i][0], c[i][1]), R, cc[i], 1, lt);
  }
}

cv::Mat make_image(const cv::Mat & img, const zarray_t * detections)
{
  cv::Mat colorImg;
  cv::cvtColor(img, colorImg, cv::COLOR_GRAY2BGR);
  const auto numDetections = zarray_size(detections);
  for (int i = 0; i < numDetections; ++i) {
    apriltag_detection_t * t;
    zarray_get(detections, i, &t);
    draw_tag(
      colorImg, t->id, {t->c[0], t->c[1]},
      *reinterpret_cast<const std::array<std::array<double, 2>, 4> *>(
        &(t->p[0][0])));
  }
  return (colorImg);
}
}  // namespace common
}  // namespace apriltag_detector_ros
