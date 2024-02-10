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
#include <tag16h5.h>
#include <tag25h9.h>
#include <tag36h11.h>
#include <tagCircle21h7.h>
#include <tagCircle49h12.h>
#include <tagCustom48h12.h>
#include <tagStandard41h12.h>
#include <tagStandard52h13.h>

#include <apriltag_detector/detector_wrapper_base.hpp>
#include <opencv2/core/core.hpp>
#include <stdexcept>

namespace apriltag_detector_ros
{

static apriltag_family * make_tag_family(const std::string & name)
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

static void destroy_tag_family(const std::string & name, apriltag_family * tf)
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

apriltag_detector_t * recast(void * p)
{
  return (reinterpret_cast<apriltag_detector_t *>(p));
}

apriltag_family_t * to_fam(void * p)
{
  return (reinterpret_cast<apriltag_family_t *>(p));
}

zarray_t * to_z(void * p) { return (reinterpret_cast<zarray_t *>(p)); }

DetectorWrapperBase::DetectorWrapperBase(const std::string & fam, int hamming)
{
  detector_ = reinterpret_cast<void *>(apriltag_detector_create());
  family_name_ = fam;
  family_ = make_tag_family(fam);
  if (!family_) {
    throw(std::runtime_error("invalid tag family specified!"));
  }
  apriltag_detector_add_family_bits(
    recast(detector_), to_fam(family_), hamming);
}

DetectorWrapperBase::~DetectorWrapperBase()
{
  if (detector_) {
    apriltag_detector_destroy(recast(detector_));
  }
  if (family_) {
    destroy_tag_family(family_name_, to_fam(family_));
  }
}

void DetectorWrapperBase::setQuadSigma(double blur)
{
  recast(detector_)->quad_sigma = blur;
}

void DetectorWrapperBase::setDecimateFactor(double decimate_factor)
{
  recast(detector_)->quad_decimate = decimate_factor;
}
void DetectorWrapperBase::setNumberOfThreads(int num_threads)
{
  recast(detector_)->nthreads = num_threads;
}

void * DetectorWrapperBase::detectTags(const cv::Mat & cvImg)
{
  image_u8_t img{
    cvImg.cols, cvImg.rows, static_cast<int32_t>(cvImg.step), cvImg.data};

  return (apriltag_detector_detect(recast(detector_), &img));
}

void DetectorWrapperBase::freeDetections(void * det)
{
  zarray_t * d = reinterpret_cast<zarray_t *>(det);
  apriltag_detections_destroy(d);
}

}  // namespace apriltag_detector_ros
