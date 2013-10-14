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
Matrix Cal3Bundler::K() const {
  Matrix3 K;
  K << f_, 0, u0_, 0, f_, v0_, 0, 0, 1;
  return K;
}

/* ************************************************************************* */
Vector Cal3Bundler::k() const {
  return Vector_(4, k1_, k2_, 0, 0);
}

/* ************************************************************************* */
Vector Cal3Bundler::vector() const {
  return Vector_(3, f_, k1_, k2_);
}

/* ************************************************************************* */
void Cal3Bundler::print(const std::string& s) const {
  gtsam::print(Vector_(5, f_, k1_, k2_, u0_, v0_), s + ".K");
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
    boost::optional<Matrix&> Dcal, boost::optional<Matrix&> Dp) const {
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
    Eigen::Matrix<double, 2, 3> D;
    D << u, f_ * rx, f_ * r * rx, v, f_ * ry, f_ * r * ry;
    *Dcal = D;
  }

  if (Dp) {
    const double a = 2. * (k1_ + 2. * k2_ * r);
    const double axx = a * x * x, axy = a * x * y, ayy = a * y * y;
    Eigen::Matrix<double, 2, 2> D;
    D << g + axx, axy, axy, g + ayy;
    *Dp = f_ * D;
  }

  return Point2(u0_ + f_ * u, v0_ + f_ * v);
}

/* ************************************************************************* */
Matrix Cal3Bundler::D2d_intrinsic(const Point2& p) const {
  Matrix Dp;
  uncalibrate(p, boost::none, Dp);
  return Dp;
}

/* ************************************************************************* */
Matrix Cal3Bundler::D2d_calibration(const Point2& p) const {
  Matrix Dcal;
  uncalibrate(p, Dcal, boost::none);
  return Dcal;
}

/* ************************************************************************* */
Matrix Cal3Bundler::D2d_intrinsic_calibration(const Point2& p) const {
  Matrix Dcal, Dp;
  uncalibrate(p, Dcal, Dp);
  Matrix H(2, 5);
  H << Dp, Dcal;
  return H;
}

/* ************************************************************************* */
Cal3Bundler Cal3Bundler::retract(const Vector& d) const {
  return Cal3Bundler(f_ + d(0), k1_ + d(1), k2_ + d(2), u0_, v0_);
}

/* ************************************************************************* */
Vector Cal3Bundler::localCoordinates(const Cal3Bundler& T2) const {
  return vector() - T2.vector();
}

}
