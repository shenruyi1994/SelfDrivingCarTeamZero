#ifndef _sdcBoundingShape_hh
#define _sdcBoundingShape_hh

#include <opencv2/opencv.hpp>

namespace gazebo {
  class sdcBoundingShape {
  public:
    cv::Point2d Centerpoint() { return centerpoint_; }
  protected:
    cv::Point2d centerpoint_;
  };
}

#endif
