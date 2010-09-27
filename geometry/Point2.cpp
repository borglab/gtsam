/**
 * @file    Point2.cpp
 * @brief   2D Point
 * @author  Frank Dellaert
 */

#include <gtsam/geometry/Point2.h>
#include <gtsam/base/Lie-inl.h>

using namespace std;

namespace gtsam {

  /** Explicit instantiation of base class to export members */
  INSTANTIATE_LIE(Point2);

  /* ************************************************************************* */
  void Point2::print(const string& s) const {
    cout << s << "(" << x_ << ", " << y_ << ")" << endl;
  }

  /* ************************************************************************* */
  bool Point2::equals(const Point2& q, double tol) const {
    return (fabs(x_ - q.x()) < tol && fabs(y_ - q.y()) < tol);
  }

  /* ************************************************************************* */
  double Point2::norm() const {
    return sqrt(x_*x_ + y_*y_);
  }

  /* ************************************************************************* */
  Point2 Point2::transform_to(const Point2& point,
  		boost::optional<Matrix&> H1, boost::optional<Matrix&> H2) const {
	  if (H1) *H1 = -eye(2);
	  if (H2) *H2 = eye(2);
	  return point - *this;
  }

  /* ************************************************************************* */
  Point2 Point2::transform_from(const Point2& point,
  	boost::optional<Matrix&> H1, boost::optional<Matrix&> H2) const {
	  if (H1) *H1 = eye(2);
	  if (H2) *H2 = eye(2);
	  return point + *this;
  }

} // namespace gtsam
