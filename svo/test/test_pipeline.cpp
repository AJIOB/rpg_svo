// This file is part of SVO - Semi-direct Visual Odometry.
//
// Copyright (C) 2014 Christian Forster <forster at ifi dot uzh dot ch>
// (Robotics and Perception Group, University of Zurich, Switzerland).
//
// SVO is free software: you can redistribute it and/or modify it under the
// terms of the GNU General Public License as published by the Free Software
// Foundation, either version 3 of the License, or any later version.
//
// SVO is distributed in the hope that it will be useful, but WITHOUT ANY
// WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
// FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.

#include <svo/config.h>
#include <svo/frame_handler_mono.h>
#include <svo/map.h>
#include <svo/frame.h>
#include <vector>
#include <string>
#include <vikit/math_utils.h>
#include <vikit/vision.h>
#include <vikit/abstract_camera.h>
#include <vikit/atan_camera.h>
#include <vikit/pinhole_camera.h>
#include <opencv2/opencv.hpp>
#include <sophus/se3.h>
#include <iostream>
#include "test_utils.h"

#define CUSTOM_DATASET_ORIGINAL ".original"
#define CUSTOM_DATASET_PATCHED ".preprocessed"
#define CUSTOM_DATASET_CANNY_PATCHED ".preprocessed_canny"

#define CUSTOM_DATASET 1
#define CUSTOM_DATASET_SELECTED CUSTOM_DATASET_CANNY_PATCHED
// Must be "1" or "2"
#define CUSTOM_DATASET_CAMERA_ID "2"
#define ALMOST_SILENT_LOGS 1
#define PRINT_POSITION_ONLY_IF_CHANGE 1

namespace svo {

class BenchmarkNode
{
  vk::AbstractCamera* cam_;
  svo::FrameHandlerMono* vo_;

public:
  BenchmarkNode();
  ~BenchmarkNode();
  void runFromFolder();
};

BenchmarkNode::BenchmarkNode()
{
#if CUSTOM_DATASET
  // fx, fy = 10 are random, but should be set as here: https://stackoverflow.com/a/16330470/6818663
  cam_ = new vk::PinholeCamera(32, 32, 10, 10, 16.0, 16.0);
#else
  cam_ = new vk::PinholeCamera(752, 480, 315.5, 315.5, 376.0, 240.0);
#endif
  vo_ = new svo::FrameHandlerMono(cam_);
  vo_->start();
}

BenchmarkNode::~BenchmarkNode()
{
  delete vo_;
  delete cam_;
}

void BenchmarkNode::runFromFolder()
{
  double xx = 0, yy = 0, zz = 0;

#if CUSTOM_DATASET
  for(int img_id = 1; img_id < 4434; ++img_id)
#else
  for(int img_id = 2; img_id < 188; ++img_id)
#endif
  {
    // load image
    std::stringstream ss;
#if CUSTOM_DATASET
    ss << svo::test_utils::getDatasetDir() << "/dataset" CUSTOM_DATASET_SELECTED "/cam." CUSTOM_DATASET_CAMERA_ID "."
       << img_id << ".data.pgm";
#else
    ss << svo::test_utils::getDatasetDir() << "/sin2_tex2_h1_v8_d/img/frame_"
       << std::setw( 6 ) << std::setfill( '0' ) << img_id << "_0.png";
#endif
    if(img_id == 2)
      std::cout << "reading image " << ss.str() << std::endl;
    cv::Mat img(cv::imread(ss.str().c_str(), 0));
    assert(!img.empty());

    // process frame
    vo_->addImage(img, 0.01*img_id);

    // display tracking quality
    if(vo_->lastFrame() != NULL)
    {
#if !ALMOST_SILENT_LOGS
    	std::cout << "Frame-Id: " << vo_->lastFrame()->id_ << " \t"
                  << "#Features: " << vo_->lastNumObservations() << " \t"
                  << "Proc. Time: " << vo_->lastProcessingTime()*1000 << "ms ";
#endif

      auto& v = vo_->lastFrame()->T_f_w_.unit_quaternion().vec();

      // std::cout << " \t" << v.x() << " \t" << v.y() << " \t" << v.z() << "\n";

      xx += v.x();
      yy += v.y();
      zz += v.z();

#if !ALMOST_SILENT_LOGS || PRINT_POSITION_ONLY_IF_CHANGE
      if ((!PRINT_POSITION_ONLY_IF_CHANGE) || v.x() || v.y() || v.z())
      {
        std::cout << " \t" << xx << " \t" << yy << " \t" << zz << "\n";
      }
#endif

    	// access the pose of the camera via vo_->lastFrame()->T_f_w_.
    }
  }

  std::cout << "Total results: \t" << xx << " \t" << yy << " \t" << zz << "\n";
}

} // namespace svo

int main(int argc, char** argv)
{
  {
    svo::BenchmarkNode benchmark;
    benchmark.runFromFolder();
  }
  printf("BenchmarkNode finished.\n");
  return 0;
}
