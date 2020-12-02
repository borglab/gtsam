/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file Cal3Bundler.cpp
 * @date Sep 25, 2010
 * @author ydjian
 */

#include <gtsam/base/Vector.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Cal3Bundler.h>

namespace gtsam {

/* ************************************************************************* */
Matrix3 Cal3Bundler::K() const {
  Matrix3 K;
  K << f_, 0, u0_, 0, f_, v0_, 0, 0, 1;
  return K;
}

/* ************************************************************************* */
Vector4 Cal3Bundler::k() const {
  Vector4 rvalue_;
  rvalue_ << k1_, k2_, 0, 0;
  return rvalue_;
}

/* ************************************************************************* */
Vector3 Cal3Bundler::vector() const {
  return Vector3(f_, k1_, k2_);
}

/* ************************************************************************* */
std::ostream& operator<<(std::ostream& os, const Cal3Bundler& cal) {
  os << "f: " << cal.fx() << ", k1: " << cal.k1() << ", k2: " << cal.k2()
     << ", px: " << cal.px() << ", py: " << cal.py();
  return os;
}

/* ************************************************************************* */
void Cal3Bundler::print(const std::string& s) const {
  gtsam::print((Vector)(Vector(5) << f_, k1_, k2_, u0_, v0_).finished(), s + ".K");
}

/* ************************************************************************* */
bool Cal3Bundler::equals(const Cal3Bundler& K, double tol) const {
  return (std::fabs(f_ - K.f_) < tol && std::fabs(k1_ - K.k1_) < tol &&
          std::fabs(k2_ - K.k2_) < tol && std::fabs(u0_ - K.u0_) < tol &&
          std::fabs(v0_ - K.v0_) < tol);
}

/* ************************************************************************* */
Point2 Cal3Bundler::uncalibrate(const Point2& p, //
    OptionalJacobian<2, 3> Dcal, OptionalJacobian<2, 2> Dp) const {
  //  r = x^2 + y^2;
  //  g = (1 + k(1)*r + k(2)*r^2);
  //  pi(:,i) = g * pn(:,i)
  const double x = p.x(), y = p.y();
  const double r = x * x + y * y;
  const double g = 1. + (k1_ + k2_ * r) * r;
  const double u = g * x, v = g * y;

  // Derivatives make use of intermediate variables above
  if (Dcal) {
    double rx = r * x, ry = r * y;
    *Dcal << u, f_ * rx, f_ * r * rx, v, f_ * ry, f_ * r * ry;
  }

  if (Dp) {
    const double a = 2. * (k1_ + 2. * k2_ * r);
    const double axx = a * x * x, axy = a * x * y, ayy = a * y * y;
    *Dp << g + axx, axy, axy, g + ayy;
    *Dp *= f_;
  }

  return Point2(u0_ + f_ * u, v0_ + f_ * v);
}

/* ************************************************************************* */
Point2 Cal3Bundler::calibrate(const Point2& pi,
                              OptionalJacobian<2, 3> Dcal,
                              OptionalJacobian<2, 2> Dp) const {
  // Copied from Cal3DS2 :-(
  // but specialized with k1,k2 non-zero only and fx=fy and s=0
  double x = (pi.x() - u0_)/f_, y = (pi.y() - v0_)/f_;
  const Point2 invKPi(x, y);

  // initialize by ignoring the distortion at all, might be problematic for pixels around boundary
  Point2 pn(x, y);

  // iterate until the uncalibrate is close to the actual pixel coordinate
  const int maxIterations = 10;
  int iteration;
  for (iteration = 0; iteration < maxIterations; ++iteration) {
    if (distance2(uncalibrate(pn), pi) <= tol_)
      break;
    const double px = pn.x(), py = pn.y(), xx = px * px, yy = py * py;
    const double rr = xx + yy;
    const double g = (1 + k1_ * rr + k2_ * rr * rr);
    pn = invKPi / g;
  }

  if (iteration >= maxIterations)
    throw std::runtime_error(
        "Cal3Bundler::calibrate fails to converge. need a better initialization");

  calibrateJacobians<Cal3Bundler, dimension>(*this, pn, Dcal, Dp);

  return pn;
}

/* ************************************************************************* */
Matrix2 Cal3Bundler::D2d_intrinsic(const Point2& p) const {
  Matrix2 Dp;
  uncalibrate(p, boost::none, Dp);
  return Dp;
}

/* ************************************************************************* */
Matrix23 Cal3Bundler::D2d_calibration(const Point2& p) const {
  Matrix23 Dcal;
  uncalibrate(p, Dcal, boost::none);
  return Dcal;
}

/* ************************************************************************* */
Matrix25 Cal3Bundler::D2d_intrinsic_calibration(const Point2& p) const {
  Matrix23 Dcal;
  Matrix2 Dp;
  uncalibrate(p, Dcal, Dp);
  Matrix25 H;
  H << Dp, Dcal;
  return H;
}

}  // \ namespace gtsam
