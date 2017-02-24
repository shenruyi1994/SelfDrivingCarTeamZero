#ifndef _sdcBoundingCircle_hh
#define _sdcBoundingCircle_hh

#include <opencv2/opencv.hpp>

#include "sdcAngle.hh"
#include "sdcBoundingShape.hh"

namespace gazebo {
  class sdcBoundingCircle: public sdcBoundingShape {
  public:
    sdcBoundingCircle(double x, double y, double radius): radius(radius) {
      center = cv::Point2d(x, y);
    }
    sdcBoundingCircle(const cv::Point2d& center, double radius):
      center(center), radius(radius) {}

    bool DoesIntersect(const sdcBoundingCircle& circle) const;

    cv::Point2d center;
    double radius;
  };
}

#endif
