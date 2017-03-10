#include "sdcRotatedBoundingBox.hh"

#include <math.h>
#include <opencv2/opencv.hpp>

#include "sdcBoundingBox.hh"
#include "sdcBoundingCircle.hh"
#include "sdcUtils.hh"

using namespace gazebo;

bool sdcRotatedBoundingBox::DoesIntersect(const sdcBoundingBox& box) const {
  sdcRotatedBoundingBox rotateBox = sdcRotatedBoundingBox(
    cv::Point2f(box.x + box.width / 2, box.y - box.height /2),
    box.width, box.height, 0);
  return DoesIntersect(rotateBox);
}

bool sdcRotatedBoundingBox::DoesIntersect(const sdcRotatedBoundingBox& box) const {
  // Finds a vector parallel to each of the for pairs of sides of the provided
  // box and the own boxes.
  cv::Point2d axes[4];
  axes[0] = rotate_point(cv::Point2d(width, 0), angle);
  axes[1] = rotate_point(cv::Point2d(0, height), angle);
  axes[0] = rotate_point(cv::Point2d(box.width, 0), box.angle);
  axes[1] = rotate_point(cv::Point2d(0, box.height), box.angle);

  // Projects the corners of each box onto each axis and checks whether they
  // overlap
  for (int i = 0; i < 4; i++) {
    if (!DoProjectionsOntoLineIntersect(axes[i], box)) {
      return false;
    }
  }
  return true;
}

bool sdcRotatedBoundingBox::DoProjectionsOntoLineIntersect(
    const cv::Point2d& axis, const sdcRotatedBoundingBox& box) const {

  // Projects each of the box's own corners onto the given axis
  double selfProjections[4];
  selfProjections[0] = axis.ddot((ur.ddot(axis) / norm2(axis)) * axis);
  selfProjections[1] = axis.ddot((ul.ddot(axis) / norm2(axis)) * axis);
  selfProjections[2] = axis.ddot((ll.ddot(axis) / norm2(axis)) * axis);
  selfProjections[3] = axis.ddot((lr.ddot(axis) / norm2(axis)) * axis);

  // Finds the minimum and maximum projections of our own corners onto the axis
  double selfMin = selfProjections[0];
  double selfMax = selfProjections[0];
  for (int i = 1; i < 4; i++) {
    if (selfProjections[i] < selfMin) {
      selfMin = selfProjections[i];
    }
    if (selfProjections[i] > selfMax) {
      selfMax = selfProjections[i];
    }
  }

  // Projects each of the other box's corners onto the given axis
  double boxProjections[4];
  boxProjections[0] = axis.ddot((box.ur.ddot(axis) / norm2(axis)) * axis);
  boxProjections[1] = axis.ddot((box.ul.ddot(axis) / norm2(axis)) * axis);
  boxProjections[2] = axis.ddot((box.ll.ddot(axis) / norm2(axis)) * axis);
  boxProjections[3] = axis.ddot((box.lr.ddot(axis) / norm2(axis)) * axis);

  // Finds the minimum and maximum projections of the other box's corners onto
  // the axis
  double boxMin = boxProjections[0];
  double boxMax = boxProjections[0];
  for (int i = 1; i < 4; i++) {
    if (boxProjections[i] < boxMin) {
      boxMin = boxProjections[i];
    }
    if (boxProjections[i] > boxMax) {
      boxMax = boxProjections[i];
    }
  }

  return selfMin <= boxMax && selfMax >= boxMin;
}

bool sdcRotatedBoundingBox::DoesIntersect(const sdcBoundingCircle& circle) const {
  // rotate coordinate system such that the box has sides parallel to the axes
  cv::Point2d c = rotate_generic(circle.center, this->center, -this->angle);

  // Find the closest point to the circle within the rectangle
  cv::Point2d closestPoint = cv::Point2d(
    clamp(c.x, this->left, this->right),
    clamp(c.y, this->top, this->bottom)
  );

  // return true if the distance from closestPoint to the center of the circle
  // is less than the radius
  return coord_distance(c, closestPoint) < circle.radius;
}
