/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

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
  cout << s << *this << endl;
}

/* ************************************************************************* */
bool Point2::equals(const Point2& q, double tol) const {
  return (fabs(x_ - q.x()) < tol && fabs(y_ - q.y()) < tol);
}

/* ************************************************************************* */
double Point2::norm() const {
  return sqrt(x_ * x_ + y_ * y_);
}

/* ************************************************************************* */
static const Matrix I2 = eye(2);
double Point2::distance(const Point2& point, boost::optional<Matrix&> H1,
    boost::optional<Matrix&> H2) const {
  Point2 d = point - *this;
  double x = d.x(), y = d.y(), d2 = x * x + y * y, r = sqrt(d2);
  Matrix D_result_d;
  if (std::abs(r) > 1e-10)
    D_result_d = Matrix_(1, 2, x / r, y / r);
  else
    D_result_d = Matrix_(1, 2, 1.0, 1.0);
  if (H1) *H1 = -D_result_d;
  if (H2) *H2 = D_result_d;
  return r;
}

/* ************************************************************************* */
ostream &operator<<(ostream &os, const Point2& p) {
  os << '(' << p.x() << ", " << p.y() << ')';
  return os;
}

} // namespace gtsam
