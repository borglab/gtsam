/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file Cal3Fisheye.cpp
 * @date Apr 8, 2020
 * @author ghaggin
 */

#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Cal3Fisheye.h>
#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/Point3.h>

namespace gtsam {

/* ************************************************************************* */
Cal3Fisheye::Cal3Fisheye(const Vector9& v)
    : fx_(v[0]),
      fy_(v[1]),
      s_(v[2]),
      u0_(v[3]),
      v0_(v[4]),
      k1_(v[5]),
      k2_(v[6]),
      k3_(v[7]),
      k4_(v[8]) {}

/* ************************************************************************* */
Vector9 Cal3Fisheye::vector() const {
  Vector9 v;
  v << fx_, fy_, s_, u0_, v0_, k1_, k2_, k3_, k4_;
  return v;
}

/* ************************************************************************* */
Matrix3 Cal3Fisheye::K() const {
  Matrix3 K;
  K << fx_, s_, u0_, 0.0, fy_, v0_, 0.0, 0.0, 1.0;
  return K;
}

/* ************************************************************************* */
double Cal3Fisheye::Scaling(double r) {
  static constexpr double threshold = 1e-8;
  if (r > threshold || r < -threshold) {
    return atan(r) / r;
  } else {
    // Taylor expansion close to 0
    double r2 = r * r, r4 = r2 * r2;
    return 1.0 - r2 / 3 + r4 / 5;
  }
}

/* ************************************************************************* */
Point2 Cal3Fisheye::uncalibrate(const Point2& p, OptionalJacobian<2, 9> H1,
                                OptionalJacobian<2, 2> H2) const {
  const double xi = p.x(), yi = p.y();
  const double r2 = xi * xi + yi * yi, r = sqrt(r2);
  const double t = atan(r);
  const double t2 = t * t, t4 = t2 * t2, t6 = t2 * t4, t8 = t4 * t4;
  Vector5 K, T;
  K << 1, k1_, k2_, k3_, k4_;
  T << 1, t2, t4, t6, t8;
  const double scaling = Scaling(r);
  const double s = scaling * K.dot(T);
  const double xd = s * xi, yd = s * yi;
  Point2 uv(fx_ * xd + s_ * yd + u0_, fy_ * yd + v0_);

  Matrix2 DK;
  if (H1 || H2) DK << fx_, s_, 0.0, fy_;

  // Derivative for calibration parameters (2 by 9)
  if (H1) {
    Matrix25 DR1;
    // order: fx, fy, s, u0, v0
    DR1 << xd, 0.0, yd, 1.0, 0.0, 0.0, yd, 0.0, 0.0, 1.0;

    // order: k1, k2, k3, k4
    Matrix24 DR2;
    auto T4 = T.tail<4>().transpose();
    DR2 << xi * T4, yi * T4;
    *H1 << DR1, DK * scaling * DR2;
  }

  // Derivative for points in intrinsic coords (2 by 2)
  if (H2) {
    const double dtd_dt =
        1 + 3 * k1_ * t2 + 5 * k2_ * t4 + 7 * k3_ * t6 + 9 * k4_ * t8;
    const double dt_dr = 1 / (1 + r2);
    const double rinv = 1 / r;
    const double dr_dxi = xi * rinv;
    const double dr_dyi = yi * rinv;
    const double dtd_dxi = dtd_dt * dt_dr * dr_dxi;
    const double dtd_dyi = dtd_dt * dt_dr * dr_dyi;

    const double td = t * K.dot(T);
    const double rrinv = 1 / r2;
    const double dxd_dxi =
        dtd_dxi * dr_dxi + td * rinv - td * xi * rrinv * dr_dxi;
    const double dxd_dyi = dtd_dyi * dr_dxi - td * xi * rrinv * dr_dyi;
    const double dyd_dxi = dtd_dxi * dr_dyi - td * yi * rrinv * dr_dxi;
    const double dyd_dyi =
        dtd_dyi * dr_dyi + td * rinv - td * yi * rrinv * dr_dyi;

    Matrix2 DR;
    DR << dxd_dxi, dxd_dyi, dyd_dxi, dyd_dyi;

    *H2 = DK * DR;
  }

  return uv;
}

/* ************************************************************************* */
Point2 Cal3Fisheye::calibrate(const Point2& uv, const double tol) const {
  // initial gues just inverts the pinhole model
  const double u = uv.x(), v = uv.y();
  const double yd = (v - v0_) / fy_;
  const double xd = (u - s_ * yd - u0_) / fx_;
  Point2 pi(xd, yd);

  // Perform newtons method, break when solution converges past tol,
  // throw exception if max iterations are reached
  const int maxIterations = 10;
  int iteration;
  for (iteration = 0; iteration < maxIterations; ++iteration) {
    Matrix2 jac;

    // Calculate the current estimate (uv_hat) and the jacobian
    const Point2 uv_hat = uncalibrate(pi, boost::none, jac);

    // Test convergence
    if ((uv_hat - uv).norm() < tol) break;

    // Newton's method update step
    pi = pi - jac.inverse() * (uv_hat - uv);
  }

  if (iteration >= maxIterations)
    throw std::runtime_error(
        "Cal3Fisheye::calibrate fails to converge. need a better "
        "initialization");

  return pi;
}

/* ************************************************************************* */
void Cal3Fisheye::print(const std::string& s_) const {
  gtsam::print((Matrix)K(), s_ + ".K");
  gtsam::print(Vector(k()), s_ + ".k");
}

/* ************************************************************************* */
bool Cal3Fisheye::equals(const Cal3Fisheye& K, double tol) const {
  if (std::abs(fx_ - K.fx_) > tol || std::abs(fy_ - K.fy_) > tol ||
      std::abs(s_ - K.s_) > tol || std::abs(u0_ - K.u0_) > tol ||
      std::abs(v0_ - K.v0_) > tol || std::abs(k1_ - K.k1_) > tol ||
      std::abs(k2_ - K.k2_) > tol || std::abs(k3_ - K.k3_) > tol ||
      std::abs(k4_ - K.k4_) > tol)
    return false;
  return true;
}

/* ************************************************************************* */
Cal3Fisheye Cal3Fisheye::retract(const Vector& d) const {
  return Cal3Fisheye(vector() + d);
}

/* ************************************************************************* */
Vector Cal3Fisheye::localCoordinates(const Cal3Fisheye& T2) const {
  return T2.vector() - vector();
}

}  // namespace gtsam
/* ************************************************************************* */
