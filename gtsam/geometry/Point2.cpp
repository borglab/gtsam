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

static const Matrix oneOne = Matrix_(1, 2, 1.0, 1.0);

/* ************************************************************************* */
void Point2::print(const string& s) const {
  cout << s << *this << endl;
}

/* ************************************************************************* */
bool Point2::equals(const Point2& q, double tol) const {
  return (fabs(x_ - q.x()) < tol && fabs(y_ - q.y()) < tol);
}

/* ************************************************************************* */
double Point2::norm(boost::optional<Matrix&> H) const {
  double r = sqrt(x_ * x_ + y_ * y_);
  if (H) {
    Matrix D_result_d;
    if (std::abs(r) > 1e-10)
      D_result_d = Matrix_(1, 2, x_ / r, y_ / r);
    else
      D_result_d = oneOne; // TODO: really infinity, why 1 here??
    *H = D_result_d;
  }
  return r;
}

/* ************************************************************************* */
static const Matrix I2 = eye(2);
double Point2::distance(const Point2& point, boost::optional<Matrix&> H1,
    boost::optional<Matrix&> H2) const {
  Point2 d = point - *this;
  if (H1 || H2) {
    Matrix H;
    double r = d.norm(H);
    if (H1) *H1 = -H;
    if (H2) *H2 =  H;
    return r;
  } else
    return d.norm();
}

/* ************************************************************************* */
ostream &operator<<(ostream &os, const Point2& p) {
  os << '(' << p.x() << ", " << p.y() << ')';
  return os;
}

} // namespace gtsam
