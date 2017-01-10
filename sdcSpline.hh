#ifndef __SPLINE_H__
#define __SPLINE_H__

#include <math.h>
#include <cstdlib>
#include <vector>

#include <opencv2/opencv.hpp>

#include "sdcAngle.hh"

class sdcSpline {
protected:
  std::vector<double> knots_;
  std::vector<cv::Point2d> controlPoints_;
  double length_;

public:
  sdcSpline() {}
  ~sdcSpline() {}

  void AddControlPoint(cv::Point2d p);

  void RemoveControlPoint(int index);

  /*
   * Resets the knot values of the spline. Because getPoint() uses four
   * nearby control points, it is impossible to interpolate between the
   * first two and last two control points. Thus the number of knots_ is two
   * smaller than the number of control points.
   */
  void ResetKnots();

  /*
   * Approximates the length of the spline. Should not be called often, as it
   * is not especially optimized and is relatively expensive.
   */
  void CalculateLength();

  /*
   * Finds the nearest two control points and interpolates using the four
   * nearest control points.
   */
  cv::Point2d GetPoint(double t) const;

  /* Estimates the angle at time t. */
  sdcAngle GetAngle(double t) const;

  /*
   * Interpolates with the function T * tau * B * P, where:
   *   T is the time vector (1, t, t^2, t^3),
   *   tau is a scaling factor,
   *   B is the basis matrix of the spline,
   *   P is the point vector (p0, p1, p2, p3), where t is in between the
   *    knot values for points p1 and p2
   */
  cv::Point2d ApplyBasisMatrix(double t,
                               cv::Point2d& p0,
                               cv::Point2d& p1,
                               cv::Point2d& p2,
                               cv::Point2d& p3) const;

  /*
   *
   */
  void UpdateDistance(double d);


  /*
   * Note that splines don't have segments between the first two or last
   * two control points.
   */
  int NumSegments() const { return controlPoints_.size() - 3; }
};

#endif
