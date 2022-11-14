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
 * @author Varun Agrawal
 */

#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Cal3Fisheye.h>
#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/Point3.h>

namespace gtsam {

/* ************************************************************************* */
Vector9 Cal3Fisheye::vector() const {
  Vector9 v;
  v << fx_, fy_, s_, u0_, v0_, k1_, k2_, k3_, k4_;
  return v;
}

/* ************************************************************************* */
Point2 Cal3Fisheye::uncalibrate(const Point2& p, OptionalJacobian<2, 9> H1,
                                OptionalJacobian<2, 2> H2) const {
  const double xi = p.x(), yi = p.y(), zi = 1;
  const double r2 = xi * xi + yi * yi, r = sqrt(r2);
  const double t = atan2(r, zi);
  const double t2 = t * t, t4 = t2 * t2, t6 = t2 * t4, t8 = t4 * t4;
  Vector5 K, T;
  K << 1, k1_, k2_, k3_, k4_;
  T << 1, t2, t4, t6, t8;
  T *= t;
  const double theta_d = K.dot(T);
  double scaling = 1.0;
  if (r > 1e-8) {
    scaling = theta_d / r;
  } else {
    // r close to 0, taylor expansion for [theta_d / r], O(r^6)
    const double r4 = r2 * r2;
    scaling = 1 - r2 / 3 + r4 / 5 + k1_ * r2 * (1 - r2 + r4 / 3) + k2_ * r4;
  }
  const double xd = scaling * xi, yd = scaling * yi;
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
    if (r > 1e-8) {
      auto T4 = T.tail<4>().transpose();
      DR2 << xi * T4 / r, yi * T4 / r;
    } else {
      DR2.setZero();
    }
    *H1 << DR1, DK * DR2;
  }

  // Derivative for points in intrinsic coords (2 by 2)
  if (H2) {
    if (r < 1e-8) {
      *H2 = DK;
    } else {
      const double dtd_dt =
          1 + 3 * k1_ * t2 + 5 * k2_ * t4 + 7 * k3_ * t6 + 9 * k4_ * t8;
      const double R2 = r2 + zi * zi;
      const double dt_dr = zi / R2;
      const double rinv = 1 / r;
      const double dr_dxi = xi * rinv;
      const double dr_dyi = yi * rinv;
      const double dtd_dr = dtd_dt * dt_dr;

      const double c2 = dr_dxi * dr_dxi;
      const double s2 = dr_dyi * dr_dyi;
      const double cs = dr_dxi * dr_dyi;

      const double dxd_dxi = dtd_dr * c2 + scaling * (1 - c2);
      const double dxd_dyi = (dtd_dr - scaling) * cs;
      const double dyd_dxi = dxd_dyi;
      const double dyd_dyi = dtd_dr * s2 + scaling * (1 - s2);

      Matrix2 DR;
      DR << dxd_dxi, dxd_dyi, dyd_dxi, dyd_dyi;

      *H2 = DK * DR;
    }
  }

  return uv;
}

/* ************************************************************************* */
Point2 Cal3Fisheye::calibrate(const Point2& uv, OptionalJacobian<2, 9> Dcal,
                              OptionalJacobian<2, 2> Dp) const {
  // Apply inverse camera matrix to map the pixel coordinate (u, v)
  // of the equidistant fisheye image to angular coordinate space (xd, yd)
  // with radius theta given in radians.
  const double u = uv.x(), v = uv.y();
  const double yd = (v - v0_) / fy_;
  const double xd = (u - s_ * yd - u0_) / fx_;

  // Provide initial guess for the Newton search.
  // The angular coordinates given by (xd, yd) are mapped back to
  // the focal plane of the perspective undistorted projection pi.
  // See Cal3Unified.calibrate() using the same pattern for the
  // undistortion of omnidirectional fisheye projection.
  Point2 pi(xd, yd);

  // Perform newtons method, break when solution converges past tol_,
  // throw exception if max iterations are reached
  const int maxIterations = 10;
  int iteration;
  for (iteration = 0; iteration < maxIterations; ++iteration) {
    Matrix2 jac;

    // Calculate the current estimate (uv_hat) and the jacobian
    const Point2 uv_hat = uncalibrate(pi, boost::none, jac);

    // Test convergence
    if ((uv_hat - uv).norm() < tol_) break;

    // Newton's method update step
    pi = pi - jac.inverse() * (uv_hat - uv);
  }

  if (iteration >= maxIterations)
    throw std::runtime_error(
        "Cal3Fisheye::calibrate fails to converge. need a better "
        "initialization");

  calibrateJacobians<Cal3Fisheye, dimension>(*this, pi, Dcal, Dp);

  return pi;
}

/* ************************************************************************* */
std::ostream& operator<<(std::ostream& os, const Cal3Fisheye& cal) {
  os << (Cal3&)cal;
  os << ", k1: " << cal.k1() << ", k2: " << cal.k2() << ", k3: " << cal.k3()
     << ", k4: " << cal.k4();
  return os;
}

/* ************************************************************************* */
void Cal3Fisheye::print(const std::string& s_) const {
  gtsam::print((Matrix)K(), s_ + ".K");
  gtsam::print(Vector(k()), s_ + ".k");
}

/* ************************************************************************* */
bool Cal3Fisheye::equals(const Cal3Fisheye& K, double tol) const {
  const Cal3* base = dynamic_cast<const Cal3*>(&K);
  return Cal3::equals(*base, tol) && std::fabs(k1_ - K.k1_) < tol &&
         std::fabs(k2_ - K.k2_) < tol && std::fabs(k3_ - K.k3_) < tol &&
         std::fabs(k4_ - K.k4_) < tol;
}

/* ************************************************************************* */

}  // namespace gtsam
