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
Cal3Bundler::Cal3Bundler() :
    f_(1), k1_(0), k2_(0), u0_(0), v0_(0) {
}

/* ************************************************************************* */
Cal3Bundler::Cal3Bundler(double f, double k1, double k2, double u0, double v0) :
    f_(f), k1_(k1), k2_(k2), u0_(u0), v0_(v0) {
}

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
void Cal3Bundler::print(const std::string& s) const {
  gtsam::print((Vector)(Vector(5) << f_, k1_, k2_, u0_, v0_).finished(), s + ".K");
}

/* ************************************************************************* */
bool Cal3Bundler::equals(const Cal3Bundler& K, double tol) const {
  if (fabs(f_ - K.f_) > tol || fabs(k1_ - K.k1_) > tol
      || fabs(k2_ - K.k2_) > tol || fabs(u0_ - K.u0_) > tol
      || fabs(v0_ - K.v0_) > tol)
    return false;
  return true;
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
Point2 Cal3Bundler::calibrate(const Point2& pi, const double tol) const {
  // Copied from Cal3DS2 :-(
  // but specialized with k1,k2 non-zero only and fx=fy and s=0
  const Point2 invKPi((pi.x() - u0_)/f_, (pi.y() - v0_)/f_);

  // initialize by ignoring the distortion at all, might be problematic for pixels around boundary
  Point2 pn = invKPi;

  // iterate until the uncalibrate is close to the actual pixel coordinate
  const int maxIterations = 10;
  int iteration;
  for (iteration = 0; iteration < maxIterations; ++iteration) {
    if (distance2(uncalibrate(pn), pi) <= tol)
      break;
    const double x = pn.x(), y = pn.y(), xx = x * x, yy = y * y;
    const double rr = xx + yy;
    const double g = (1 + k1_ * rr + k2_ * rr * rr);
    pn = invKPi / g;
  }

  if (iteration >= maxIterations)
    throw std::runtime_error(
        "Cal3DS2::calibrate fails to converge. need a better initialization");

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

/* ************************************************************************* */
Cal3Bundler Cal3Bundler::retract(const Vector& d) const {
  return Cal3Bundler(f_ + d(0), k1_ + d(1), k2_ + d(2), u0_, v0_);
}

/* ************************************************************************* */
Vector3 Cal3Bundler::localCoordinates(const Cal3Bundler& T2) const {
  return T2.vector() - vector();
}

}
