#ifndef _sdcRotatedBoundingBox_hh
#define _sdcRotatedBoundingBox_hh

#include <opencv2/opencv.hpp>

#include "sdcAngle.hh"
#include "sdcBoundingBox.hh"
#include "sdcBoundingShape.hh"

namespace gazebo {
  class sdcRotatedBoundingBox: public sdcBoundingShape {
  public:
    cv::Point2d ur;
    cv::Point2d ul;
    cv::Point2d ll;
    cv::Point2d lr;

    cv::Point2d center;
    double width;
    double height;
    double angle;

    sdcRotatedBoundingBox(double left, double top, double height, double width,
                          double angle): angle(angle) {
      center = cv::Point2d(left + height / 2, top - height / 2);
      ur = cv::Point2d(left + width, top);
      ul = cv::Point2d(left, top);
      ll = cv::Point2d(left, top - height);
      lr = cv::Point2d(left + width, top - height);
    }

    sdcRotatedBoundingBox(cv::Point2d center, double width, double height,
                          double angle): center(center), angle(angle) {
      ur = cv::Point2d(center.x + width / 2, center.y + height / 2);
      ul = cv::Point2d(center.x - width / 2, center.y + height / 2);
      ll = cv::Point2d(center.x - width / 2, center.y - height / 2);
      lr = cv::Point2d(center.x + width / 2, center.y - height / 2);
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
     * Projects each corner of the two rotated bounding boxes onto the axis. If
     * the range described by the projections from one rectangle onto the other
     * do not overlap the other one, then the boxes do not intersect.
     */
    bool DoProjectionsOntoLineIntersect(const cv::Point2d& axis,
                                        const sdcRotatedBoundingBox& box) const;
  };
}

#endif
