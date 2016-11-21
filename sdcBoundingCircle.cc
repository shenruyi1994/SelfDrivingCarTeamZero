#include "sdcBoundingCircle.hh"

#include <math.h>
#include <opencv2/opencv.hpp>

#include "sdcUtils.hh"

using namespace gazebo;

bool sdcBoundingCircle::DoesIntersect(const sdcBoundingCircle& circle) const {
  return norm2(circle.center - center) <= circle.radius + radius;
}
