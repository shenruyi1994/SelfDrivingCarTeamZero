#include <type_traits>
#include <math.h>

// computes the pythagorean theoram (for distance calculation), and only
// accepts numeric types.
template<typename T>
T pythag_thm(T x, T y) {
  static_assert(std::is_arithmetic<T>::value, "pythag_thm only accepts numeric values");
  return sqrt(pow(x, 2) + pow(y, 2));
}

