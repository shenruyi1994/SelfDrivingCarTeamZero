#ifndef _sdcBoundingBox_hh
#define _sdcBoundingBox_hh

#include <opencv2/opencv.hpp>

#include "sdcAngle.hh"
#include "sdcBoundingShape.hh"

typedef cv::Rect_<double> Rect;

namespace gazebo {
  class sdcBoundingBox: public sdcBoundingShape, public Rect {
  public:
    sdcBoundingBox(double left, double top, double height, double width):
        Rect(left, top, height, width) {}

    bool DoesIntersect(const sdcBoundingBox& box) const;
  };
}

#endif