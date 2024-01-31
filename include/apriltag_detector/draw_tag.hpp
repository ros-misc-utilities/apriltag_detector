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

#ifndef APRILTAG_DETECTOR__DRAW_TAG_HPP_
#define APRILTAG_DETECTOR__DRAW_TAG_HPP_

#include <array>

namespace cv
{
class Mat;  // forward decl
}
namespace apriltag_detector
{
namespace draw_tag
{
void draw(
  cv::Mat & img, int id, const std::array<double, 2> & a,
  const std::array<std::array<double, 2>, 4> & c);

}  // namespace draw_tag
}  // namespace apriltag_detector
#endif  // APRILTAG_DETECTOR__DRAW_TAG_HPP_
