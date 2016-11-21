#include <type_traits>
#include <math.h>

#include <opencv2/opencv.hpp>

// computes the pythagorean theoram (for distance calculation), and only
// accepts numeric types.
template<typename T>
T pythag_thm(T x, T y) {
  static_assert(std::is_arithmetic<T>::value,
                "pythag_thm only accepts numeric values");
  return sqrt(pow(x, 2) + pow(y, 2));
}

template<typename T>
cv::Point_<T> rotatePoint(cv::Point_<T> point, const cv::Point_<T>& axis,
                          T angle) {
  static_assert(std::is_arithmetic<T>::value,
                "rotatePoint only accepts Points with numeric values");
  // create a new point, translated to rotate about the origin
  cv::Point_<T> newPoint = point - axis;
  float s = sin(angle);
  float c = cos(angle);

  // rotate point by angle about the origin, and translate back
  newPoint.x = (newPoint.x * c - newPoint.y * s) + axis.x;
  newPoint.y = (newPoint.x * s + newPoint.y * c) + axis.y;

  return newPoint;
}
