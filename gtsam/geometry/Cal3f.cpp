/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010
 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file   Cal3f.cpp
 * @brief  Implementation file for Cal3f class
 * @author Frank Dellaert
 * @date   October 2024
 */

#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Cal3f.h>

#include <iostream>
#include <cassert>

namespace gtsam {

/* ************************************************************************* */
Cal3f::Cal3f(double f, double u0, double v0) : Cal3(f, f, 0.0, u0, v0) {}

/* ************************************************************************* */
std::ostream& operator<<(std::ostream& os, const Cal3f& cal) {
  os << "f: " << cal.fx_ << ", px: " << cal.u0_ << ", py: " << cal.v0_;
  return os;
}

/* ************************************************************************* */
void Cal3f::print(const std::string& s) const {
  if (!s.empty()) std::cout << s << " ";
  std::cout << *this << std::endl;
}

/* ************************************************************************* */
bool Cal3f::equals(const Cal3f& K, double tol) const {
  return Cal3::equals(static_cast<const Cal3&>(K), tol);
}

/* ************************************************************************* */
Vector1 Cal3f::vector() const {
  Vector1 v;
  v << fx_;
  return v;
}

/* ************************************************************************* */
Point2 Cal3f::uncalibrate(const Point2& p, OptionalJacobian<2, 1> Dcal,
                          OptionalJacobian<2, 2> Dp) const {
  assert(fx_ == fy_ && s_ == 0.0);
  const double x = p.x(), y = p.y();
  const double u = fx_ * x + u0_;
  const double v = fx_ * y + v0_;

  if (Dcal) {
    Dcal->resize(2, 1);
    (*Dcal) << x, y;
  }

  if (Dp) {
    Dp->resize(2, 2);
    (*Dp) << fx_, 0.0,  //
        0.0, fx_;
  }

  return Point2(u, v);
}

/* ************************************************************************* */
Point2 Cal3f::calibrate(const Point2& pi, OptionalJacobian<2, 1> Dcal,
                        OptionalJacobian<2, 2> Dp) const {
  assert(fx_ == fy_ && s_ == 0.0);
  const double u = pi.x(), v = pi.y();
  double delta_u = u - u0_, delta_v = v - v0_;
  double inv_f = 1.0 / fx_;
  Point2 point(inv_f * delta_u, inv_f * delta_v);

  if (Dcal) {
    Dcal->resize(2, 1);
    (*Dcal) << -inv_f * point.x(), -inv_f * point.y();
  }

  if (Dp) {
    Dp->resize(2, 2);
    (*Dp) << inv_f, 0.0,  //
        0.0, inv_f;
  }

  return point;
}

}  // namespace gtsam