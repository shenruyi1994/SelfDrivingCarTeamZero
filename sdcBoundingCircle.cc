#include "sdcBoundingCircle.hh"

#include <math.h>
#include <opencv2/opencv.hpp>

#include "sdcUtils.hh"

using namespace gazebo;

bool sdcBoundingCircle::DoesIntersect(const sdcBoundingCircle& circle) const {
  // printf("  circle: (%f, %f)\n", circle.center.x, circle.center.y);
  // printf("  center: (%f, %f)\n", center.x, center.y);
  // printf("  obj:    %f\n", circle.radius);
  // printf("  car:    %f\n\n", radius);
  return coord_distance(circle.center, center) <= circle.radius + radius;
}
