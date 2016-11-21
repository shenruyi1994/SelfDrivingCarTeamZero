#include "sdcBoundingBox.hh"

#include <math.h>
#include <opencv2/opencv.hpp>

using namespace gazebo;
bool sdcBoundingBox::DoesIntersect(const sdcBoundingBox* box) const {
  bool intersectHoriz = (box->x <= x + width && box->x + box->width >= x)
    || (box->x >= x && box->x + box->width <= x + width);
  bool intersectVert = (box->y >= y - height && box->y - box->height <= y)
    || (box->y <= y && box->y - box->height >= y - height);
  return intersectHoriz && intersectVert;
}
