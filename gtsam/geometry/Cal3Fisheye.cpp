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
Cal3Fisheye::Cal3Fisheye(const Vector& v)
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
static Matrix29 D2dcalibration(const double xd, const double yd,
                               const double xi, const double yi,
                               const double t3, const double t5,
                               const double t7, const double t9, const double r,
                               Matrix2& DK) {
  // order: fx, fy, s, u0, v0
  Matrix25 DR1;
  DR1 << xd, 0.0, yd, 1.0, 0.0, 0.0, yd, 0.0, 0.0, 1.0;

  // order: k1, k2, k3, k4
  Matrix24 DR2;
  DR2 << t3 * xi, t5 * xi, t7 * xi, t9 * xi, t3 * yi, t5 * yi, t7 * yi, t9 * yi;
  DR2 /= r;
  Matrix29 D;
  D << DR1, DK * DR2;
  return D;
}

/* ************************************************************************* */
static Matrix2 D2dintrinsic(const double xi, const double yi, const double r,
                            const double td, const double t, const double tt,
                            const double t4, const double t6, const double t8,
                            const double k1, const double k2, const double k3,
                            const double k4, const Matrix2& DK) {
  const double dr_dxi = xi / sqrt(xi * xi + yi * yi);
  const double dr_dyi = yi / sqrt(xi * xi + yi * yi);
  const double dt_dr = 1 / (1 + r * r);
  const double dtd_dt =
      1 + 3 * k1 * tt + 5 * k2 * t4 + 7 * k3 * t6 + 9 * k4 * t8;
  const double dtd_dxi = dtd_dt * dt_dr * dr_dxi;
  const double dtd_dyi = dtd_dt * dt_dr * dr_dyi;

  const double rinv = 1 / r;
  const double rrinv = 1 / (r * r);
  const double dxd_dxi =
      dtd_dxi * xi * rinv + td * rinv + td * xi * (-rrinv) * dr_dxi;
  const double dxd_dyi = dtd_dyi * xi * rinv - td * xi * rrinv * dr_dyi;
  const double dyd_dxi = dtd_dxi * yi * rinv - td * yi * rrinv * dr_dxi;
  const double dyd_dyi =
      dtd_dyi * yi * rinv + td * rinv + td * yi * (-rrinv) * dr_dyi;

  Matrix2 DR;
  DR << dxd_dxi, dxd_dyi, dyd_dxi, dyd_dyi;

  return DK * DR;
}

/* ************************************************************************* */
Point2 Cal3Fisheye::uncalibrate(const Point2& p, OptionalJacobian<2, 9> H1,
                                OptionalJacobian<2, 2> H2) const {
  const double xi = p.x(), yi = p.y();
  const double r = sqrt(xi * xi + yi * yi);
  const double t = atan(r);
  const double tt = t * t, t4 = tt * tt, t6 = tt * t4, t8 = t4 * t4;
  const double td = t * (1 + k1_ * tt + k2_ * t4 + k3_ * t6 + k4_ * t8);
  const double td_o_r = r > 1e-8 ? td / r : 1;
  const double xd = td_o_r * xi, yd = td_o_r * yi;
  Point2 uv(fx_ * xd + s_ * yd + u0_, fy_ * yd + v0_);

  Matrix2 DK;
  if (H1 || H2) DK << fx_, s_, 0.0, fy_;

  // Derivative for calibration parameters (2 by 9)
  if (H1)
    *H1 = D2dcalibration(xd, yd, xi, yi, t * tt, t * t4, t * t6, t * t8, r, DK);

  // Derivative for points in intrinsic coords (2 by 2)
  if (H2)
    *H2 =
        D2dintrinsic(xi, yi, r, td, t, tt, t4, t6, t8, k1_, k2_, k3_, k4_, DK);

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
Matrix2 Cal3Fisheye::D2d_intrinsic(const Point2& p) const {
  const double xi = p.x(), yi = p.y();
  const double r = sqrt(xi * xi + yi * yi);
  const double t = atan(r);
  const double tt = t * t, t4 = tt * tt, t6 = t4 * tt, t8 = t4 * t4;
  const double td = t * (1 + k1_ * tt + k2_ * t4 + k3_ * t6 + k4_ * t8);

  Matrix2 DK;
  DK << fx_, s_, 0.0, fy_;

  return D2dintrinsic(xi, yi, r, td, t, tt, t4, t6, t8, k1_, k2_, k3_, k4_, DK);
}

/* ************************************************************************* */
Matrix29 Cal3Fisheye::D2d_calibration(const Point2& p) const {
  const double xi = p.x(), yi = p.y();
  const double r = sqrt(xi * xi + yi * yi);
  const double t = atan(r);
  const double tt = t * t, t4 = tt * tt, t6 = tt * t4, t8 = t4 * t4;
  const double td = t * (1 + k1_ * tt + k2_ * t4 + k3_ * t6 + k4_ * t8);
  const double xd = td / r * xi, yd = td / r * yi;

  Matrix2 DK;
  DK << fx_, s_, 0.0, fy_;
  return D2dcalibration(xd, yd, xi, yi, t * tt, t * t4, t * t6, t * t8, r, DK);
}

/* ************************************************************************* */
void Cal3Fisheye::print(const std::string& s_) const {
  gtsam::print((Matrix)K(), s_ + ".K");
  gtsam::print(Vector(k()), s_ + ".k");
  ;
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
