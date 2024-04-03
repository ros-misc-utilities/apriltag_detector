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

#include <apriltag_detector/draw_tag.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

namespace apriltag_detector
{
namespace draw_tag
{
static void draw_line(
  cv::Mat & img, double x0, double y0, double x1, double y1,
  const cv::Scalar & col, int lw)
{
  cv::line(
    img, {static_cast<int>(x0), static_cast<int>(y0)},
    {static_cast<int>(x1), static_cast<int>(y1)}, col, lw);
}

void draw(
  cv::Mat & img, int id, const std::array<double, 2> & a,
  const std::array<std::array<double, 2>, 4> & c)
{
  int lw = 2;                                   // line width
  const auto X_COLOR = CV_RGB(255, 0, 0);       // red
  const auto Y_COLOR = CV_RGB(0, 255, 0);       // green
  const auto Z_COLOR = CV_RGB(0, 0, 255);       // blue
  const auto TEXT_COLOR = CV_RGB(255, 0, 255);  // purple
  draw_line(img, c[0][0], c[0][1], c[1][0], c[1][1], X_COLOR, 2 * lw);
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

}  // namespace draw_tag
}  // namespace apriltag_detector
