#ifndef _sdcRotatedBoundingBox_hh
#define _sdcRotatedBoundingBox_hh

#include <opencv2/opencv.hpp>

#include "sdcAngle.hh"
#include "sdcBoundingBox.hh"
#include "sdcBoundingCircle.hh"
#include "sdcBoundingShape.hh"

namespace gazebo {
  class sdcRotatedBoundingBox: public sdcBoundingShape {
  public:
    cv::Point2d ur;
    cv::Point2d ul;
    cv::Point2d ll;
    cv::Point2d lr;

    double right;
    double left;
    double top;
    double bottom;

    cv::Point2d center;
    double width;
    double height;
    double angle;

    sdcRotatedBoundingBox(double left, double top, double width, double height,
                          double angle):
        left(left), top(top), angle(angle), width(width), height(height) {
      center = cv::Point2d(left + height / 2, top - height / 2);
      this->right = left + width;
      this->bottom = top - height;

      ur = cv::Point2d(right, top);
      ul = cv::Point2d(left, top);
      ll = cv::Point2d(left, bottom);
      lr = cv::Point2d(right, bottom);
    }

    sdcRotatedBoundingBox(cv::Point2d center, double width, double height,
                          double angle):
        center(center), angle(angle), width(width), height(height) {
      right = center.x + width / 2;
      left = center.x - width / 2;
      top = center.y + height / 2;
      bottom = center.y - height / 2;

      ur = cv::Point2d(right, top);
      ul = cv::Point2d(left, top);
      ll = cv::Point2d(left, bottom);
      lr = cv::Point2d(left, bottom);
    }

    /*
     * Creates a set of 4 axes, each linearly independent and parallel to
     * a side of one of the rectangles. These axes can then be used to
     * determine if the two boxes intersect.
     * Returns whether the given bounding box intersects with this one.
     */
    bool DoesIntersect(const sdcRotatedBoundingBox& box) const;
    bool DoesIntersect(const sdcBoundingBox& box) const;

    /*
     * Rotates the coordinate system by the bounding box's angle, then uses
     * clamp to find the closest point
     * Returns true if the circle intersects the box.
     */
    bool DoesIntersect(const sdcBoundingCircle& circle) const;

    /*
     * Projects each corner of the two rotated bounding boxes onto the axis. If
     * the range described by the projections from one rectangle onto the other
     * do not overlap the other one, then the boxes do not intersect.
     * Returns whether the ranges overlap.
     */
    bool DoProjectionsOntoLineIntersect(const cv::Point2d& axis,
                                        const sdcRotatedBoundingBox& box) const;
  };
}

#endif
