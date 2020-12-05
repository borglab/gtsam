/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file   Cal3_S2Stereo.cpp
 * @brief  The most common 5DOF 3D->2D calibration + Stereo baseline
 * @author Chris Beall
 */

#include <gtsam/geometry/Cal3_S2Stereo.h>

#include <iostream>

namespace gtsam {

/* ************************************************************************* */
std::ostream& operator<<(std::ostream& os, const Cal3_S2Stereo& cal) {
  os << (Cal3_S2&)cal;
  os << ", b: " << cal.baseline();
  return os;
}

/* ************************************************************************* */
void Cal3_S2Stereo::print(const std::string& s) const {
  std::cout << s << (s != "" ? " " : "");
  std::cout << "K: " << (Matrix)K() << std::endl;
  std::cout << "Baseline: " << b_ << std::endl;
}

/* ************************************************************************* */
bool Cal3_S2Stereo::equals(const Cal3_S2Stereo& other, double tol) const {
  const Cal3_S2* base = dynamic_cast<const Cal3_S2*>(&other);
  return (Cal3_S2::equals(*base, tol) &&
          std::fabs(b_ - other.baseline()) < tol);
}

/* ************************************************************************* */
Point2 Cal3_S2Stereo::uncalibrate(const Point2& p, OptionalJacobian<2, 6> Dcal,
                            OptionalJacobian<2, 2> Dp) const {
  const double x = p.x(), y = p.y();
  if (Dcal) *Dcal << x, 0.0, y, 1.0, 0.0, 0.0, 0.0, y, 0.0, 0.0, 1.0, 0.0;
  if (Dp) *Dp << fx_, s_, 0.0, fy_;
  return Point2(fx_ * x + s_ * y + u0_, fy_ * y + v0_);
}

/* ************************************************************************* */
Point2 Cal3_S2Stereo::calibrate(const Point2& p, OptionalJacobian<2, 6> Dcal,
                          OptionalJacobian<2, 2> Dp) const {
  const double u = p.x(), v = p.y();
  double delta_u = u - u0_, delta_v = v - v0_;
  double inv_fx = 1 / fx_, inv_fy = 1 / fy_;
  double inv_fy_delta_v = inv_fy * delta_v;
  double inv_fx_s_inv_fy = inv_fx * s_ * inv_fy;

  Point2 point(inv_fx * (delta_u - s_ * inv_fy_delta_v), inv_fy_delta_v);
  if (Dcal) {
    *Dcal << -inv_fx * point.x(), inv_fx * s_ * inv_fy * inv_fy_delta_v,
        -inv_fx * point.y(), -inv_fx, inv_fx_s_inv_fy, 0, 0,
        -inv_fy * point.y(), 0, 0, -inv_fy, 0;
  }
  if (Dp) *Dp << inv_fx, -inv_fx_s_inv_fy, 0, inv_fy;
  return point;
}

/* ************************************************************************* */

}  // namespace gtsam
