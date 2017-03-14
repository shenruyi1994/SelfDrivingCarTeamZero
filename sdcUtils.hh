#ifndef __SDCUTILS_HH__
#define __SDCUTILS_HH__

#include <type_traits>
#include <math.h>

#include <opencv2/opencv.hpp>
#include "gazebo/physics/physics.hh"

#include "Waypoints.hh"

/*
 * Clamps v to be within the range [min, max]
 */
template<typename T>
T clamp(T v, T min, T max) {
  static_assert(std::is_arithmetic<T>::value,
                "clamp only accepts numeric values");
  const T t = v < min ? min : v;
  return t > max ? max : t;
}

/*
 * Computes the pythagorean theorem (for distance calculation), and only
 * accepts numeric types.
 */
template<typename T>
T pythag_thm(T x, T y) {
  static_assert(std::is_arithmetic<T>::value,
                "pythag_thm only accepts numeric values");
  return sqrt(pow(x, 2) + pow(y, 2));
}

/*
 * Computes the distance between two points. The passed in types must each have
 * the public numeric fields x and y.
 */
template<typename T1, typename T2>
double coord_distance(const T1& p1, const T2& p2) {
  return pythag_thm(p1.x - p2.x, p1.y - p2.y);
}

/////////////////////////////////////////
// Linear Algebra functions for OpenCV //
/////////////////////////////////////////

/*
 * Returns a point rotated about axis by angle.
 */
template<typename T_1, typename T_2, typename T_3>
T_1 rotate_generic(T_1 point, const T_2& axis, T_3 angle) {
  static_assert(std::is_arithmetic<T_3>::value,
                "rotatePoint only accepts Points with numeric values");
  double s = sin(angle);
  double c = cos(angle);

  // translate point to rotate about the origin
  point.x = point.x - axis.x;
  point.y = point.y - axis.y;

  // rotate point by angle about the origin, and translate back
  point.x = (point.x * c - point.y * s) + axis.x;
  point.y = (point.x * s + point.y * c) + axis.y;

  return point;
}

/*
 * Returns a point rotated about the origin by angle.
 */
template<typename T_1, typename T_2>
T_1 rotate_generic(T_1 point, T_2 angle) {
  static_assert(std::is_arithmetic<T_2>::value,
                "rotatePoint only accepts Points with numeric values");
  double s = sin(angle);
  double c = cos(angle);

  // rotate point by angle about the origin
  point.x = (point.x * c - point.y * s);
  point.y = (point.x * s + point.y * c);

  return point;
}

/*
 * Returns the length of a point (that is, its distance from the origin).
 */
template<typename T>
T norm2(cv::Point_<T> point) {
  static_assert(std::is_arithmetic<T>::value,
                "norm only accepts Vecs with numeric values");
  return pythag_thm(point.x, point.y);
}

//////////////////////
// Type conversions //
//////////////////////

/*
 * Converts a math::Vector2d to a cv::Point2d
 */
template<typename T>
cv::Point2d to_point(const T& vec) {
  return cv::Point2d(vec.x, vec.y);
}

/*
 * Converts a cv::Point2d to a math::Vector2d
 */
template<typename T>
gazebo::math::Vector2d to_math_vec(const T& point) {
  return gazebo::math::Vector2d(point.x, point.y);
}

#endif
