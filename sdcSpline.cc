#include "sdcSpline.hh"

#include <math.h>
#include <cstdlib>
#include <vector>

#include <opencv2/opencv.hpp>

#include "sdcAngle.hh"
#include "sdcUtils.hh"

void sdcSpline::AddControlPoint(cv::Point2d p) {
  controlPoints_.push_back(p);
  ResetKnots();
}

void sdcSpline::RemoveControlPoint(int index) {
  controlPoints_.erase(controlPoints_.begin() + index);
  ResetKnots();
  CalculateLength();
}

/*
 * Resets the knot values of the spline. Because getPoint() uses four
 * nearby control points, it is impossible to interpolate between the
 * first two and last two control points. Thus the number of knots_ is two
 * smaller than the number of control points.
 */
void sdcSpline::ResetKnots() {
  knots_.clear();
  if (controlPoints_.size() > 3) {
    for (unsigned int i = 0; i < controlPoints_.size() - 2; i++) {
      double ti = (double)i / (controlPoints_.size() - 3);
      knots_.push_back(ti);
    }
  }
}

/*
 * Approximates the length of the spline. Should not be called often, as it
 * is not especially optimized and is relatively expensive.
 */
void sdcSpline::CalculateLength() {
  length_ = 0;
  cv::Point2d lastPoint = sdcSpline::GetPoint(0);
  for (int i = 1; i < 100; i++) {
    cv::Point2d curPoint = GetPoint(((int)i) / 100);
    // length_ += cv_distance(curPoint, lastPoint);
    lastPoint = curPoint;
  }
}

/*
 * Returns the next point based on the current distance travelled along the
 * spline.
 */
void sdcSpline::GetPointAtDistance(double d) {
  GetPoint(d / length_);
}

/*
 * Finds the nearest two control points and interpolates using the four
 * nearest control points.
 */
cv::Point2d sdcSpline::GetPoint(double t) const {
  if (knots_.size() < 2) {
    return cv::Point2d(0.0, 0.0);
  }

  cv::Point2d p0(0.0, 0.0), p1(0.0, 0.0), p2(0.0, 0.0), p3(0.0, 0.0);

  // Finds the two control points t is between, which are then used
  // for interpolation.
  // Next it maps t: [knots_.at(i), knots_.at(i+1)) -> [0,1).
  double temp = 0;
  for (unsigned int i = 0; i < knots_.size() - 1; ++i) {
    if (t >= knots_.at(i) && t < knots_.at(i + 1)) {
      temp = (t - knots_.at(i)) / (knots_.at(i + 1) - knots_.at(i));

      p0 = controlPoints_.at(i);
      p1 = controlPoints_.at(i + 1);
      p2 = controlPoints_.at(i + 2);
      p3 = controlPoints_.at(i + 3);
    }
  }
  t = temp;

  return ApplyBasisMatrix(t, p0, p1, p2, p3);
}

/* Estimates the angle at time t. */
sdcAngle sdcSpline::GetAngle(double t) const {
  cv::Point2d before = GetPoint(t - .001);
  cv::Point2d after = GetPoint(t + .001);

  return sdcAngle(tan((after.y - before.y) / (after.x - before.x)));
}

/*
 * Interpolates with the function T * tau * B * P, where:
 *   T is the time vector (1, t, t^2, t^3),
 *   tau is a scaling factor,
 *   B is the basis matrix of the spline,
 *   P is the point vector (p0, p1, p2, p3), where t is in between the
 *    knot values for points p1 and p2
 */
cv::Point2d sdcSpline::ApplyBasisMatrix(double t,
                                        cv::Point2d& p0,
                                        cv::Point2d& p1,
                                        cv::Point2d& p2,
                                        cv::Point2d& p3) const {
  return
    cv::Point2d(
      ((          p1*2.0              )
     + (-p0              + p2         ) * t
     + ( p0*2.0 - p1*5.0 + p2*4.0 - p3) * pow(t, 2)
     + (-p0     + p1*3.0 - p2*3.0 + p3) * pow(t, 3)
      ) * 0.5f
    );
}
