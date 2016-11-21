#include "sdcRotatedBoundingBox.hh"

#include <math.h>
#include <opencv2/opencv.hpp>

#include "sdcBoundingBox.hh"
#include "sdcUtils.hh"

using namespace gazebo;

bool sdcRotatedBoundingBox::DoesIntersect(const sdcBoundingBox& box) const {
  sdcRotatedBoundingBox rotateBox = sdcRotatedBoundingBox(
    cv::Point2f(box.x + box.width / 2, box.y - box.height /2),
    box.width, box.height, 0);
  return DoesIntersect(rotateBox);
}

bool sdcRotatedBoundingBox::DoesIntersect(const sdcRotatedBoundingBox& box) const {
  cv::Point2d axes[4];
  axes[0] = rotatePoint(cv::Point2d(width, 0), angle);
  axes[1] = rotatePoint(cv::Point2d(0, height), angle);
  axes[0] = rotatePoint(cv::Point2d(box.width, 0), box.angle);
  axes[1] = rotatePoint(cv::Point2d(0, box.height), box.angle);

  for (int i = 0; i < 4; i++) {
    if (!DoProjectionsOntoLineIntersect(axes[i], box)) {
      return false;
    }
  }
  return true;
}

bool sdcRotatedBoundingBox::DoProjectionsOntoLineIntersect(
    const cv::Point2d& axis, const sdcRotatedBoundingBox& box) const {
  double selfProjections[4];
  selfProjections[0] = axis.ddot((ur.ddot(axis) / norm2(axis)) * axis);
  selfProjections[1] = axis.ddot((ul.ddot(axis) / norm2(axis)) * axis);
  selfProjections[2] = axis.ddot((ll.ddot(axis) / norm2(axis)) * axis);
  selfProjections[3] = axis.ddot((lr.ddot(axis) / norm2(axis)) * axis);

  double selfMin = selfProjections[0];
  for (int i = 1; i < 4; i++) {
    if (selfProjections[i] < selfMin) {
      selfMin = selfProjections[i];
    }
  }
  double selfMax = selfProjections[0];
  for (int i = 1; i < 4; i++) {
    if (selfProjections[i] > selfMax) {
      selfMax = selfProjections[i];
    }
  }

  double boxProjections[4];
  boxProjections[0] = axis.ddot((box.ur.ddot(axis) / norm2(axis)) * axis);
  boxProjections[1] = axis.ddot((box.ul.ddot(axis) / norm2(axis)) * axis);
  boxProjections[2] = axis.ddot((box.ll.ddot(axis) / norm2(axis)) * axis);
  boxProjections[3] = axis.ddot((box.lr.ddot(axis) / norm2(axis)) * axis);

  double boxMin = boxProjections[0];
  for (int i = 1; i < 4; i++) {
    if (boxProjections[i] < boxMin) {
      boxMin = boxProjections[i];
    }
  }
  double boxMax = boxProjections[0];
  for (int i = 1; i < 4; i++) {
    if (boxProjections[i] > boxMax) {
      boxMax = boxProjections[i];
    }
  }

  return selfMin <= boxMax && selfMax >= boxMin;
}
