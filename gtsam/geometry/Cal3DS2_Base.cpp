/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file Cal3DS2_Base.cpp
 * @date Feb 28, 2010
 * @author ydjian
 */

#include <gtsam/base/Vector.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Cal3DS2_Base.h>

namespace gtsam {

/* ************************************************************************* */
Cal3DS2_Base::Cal3DS2_Base(const Vector &v):
    fx_(v[0]), fy_(v[1]), s_(v[2]), u0_(v[3]), v0_(v[4]), k1_(v[5]), k2_(v[6]), p1_(v[7]), p2_(v[8]){}

/* ************************************************************************* */
Matrix3 Cal3DS2_Base::K() const {
    Matrix3 K;
    K << fx_, s_, u0_, 0.0, fy_, v0_, 0.0, 0.0, 1.0;
    return K;
}

/* ************************************************************************* */
Vector9 Cal3DS2_Base::vector() const {
  Vector9 v;
  v << fx_, fy_, s_, u0_, v0_, k1_, k2_, p1_, p2_;
  return v;
}

/* ************************************************************************* */
void Cal3DS2_Base::print(const std::string& s_) const {
  gtsam::print((Matrix)K(), s_ + ".K");
  gtsam::print(Vector(k()), s_ + ".k");
}

/* ************************************************************************* */
bool Cal3DS2_Base::equals(const Cal3DS2_Base& K, double tol) const {
  if (fabs(fx_ - K.fx_) > tol || fabs(fy_ - K.fy_) > tol || fabs(s_ - K.s_) > tol ||
      fabs(u0_ - K.u0_) > tol || fabs(v0_ - K.v0_) > tol || fabs(k1_ - K.k1_) > tol ||
      fabs(k2_ - K.k2_) > tol || fabs(p1_ - K.p1_) > tol || fabs(p2_ - K.p2_) > tol)
    return false;
  return true;
}

/* ************************************************************************* */
static Matrix29 D2dcalibration(double x, double y, double xx,
    double yy, double xy, double rr, double r4, double pnx, double pny,
    const Matrix2& DK) {
  Matrix25 DR1;
  DR1 << pnx, 0.0, pny, 1.0, 0.0, 0.0, pny, 0.0, 0.0, 1.0;
  Matrix24 DR2;
  DR2 << x * rr, x * r4, 2 * xy, rr + 2 * xx, //
         y * rr, y * r4, rr + 2 * yy, 2 * xy;
  Matrix29 D;
  D << DR1, DK * DR2;
  return D;
}

/* ************************************************************************* */
static Matrix2 D2dintrinsic(double x, double y, double rr,
    double g, double k1, double k2, double p1, double p2,
    const Matrix2& DK) {
  const double drdx = 2. * x;
  const double drdy = 2. * y;
  const double dgdx = k1 * drdx + k2 * 2. * rr * drdx;
  const double dgdy = k1 * drdy + k2 * 2. * rr * drdy;

  // Dx = 2*p1*xy + p2*(rr+2*xx);
  // Dy = 2*p2*xy + p1*(rr+2*yy);
  const double dDxdx = 2. * p1 * y + p2 * (drdx + 4. * x);
  const double dDxdy = 2. * p1 * x + p2 * drdy;
  const double dDydx = 2. * p2 * y + p1 * drdx;
  const double dDydy = 2. * p2 * x + p1 * (drdy + 4. * y);

  Matrix2 DR;
  DR << g + x * dgdx + dDxdx, x * dgdy + dDxdy, //
        y * dgdx + dDydx, g + y * dgdy + dDydy;

  return DK * DR;
}

/* ************************************************************************* */
Point2 Cal3DS2_Base::uncalibrate(const Point2& p,
    OptionalJacobian<2,9> H1, OptionalJacobian<2,2> H2) const {

  //  rr = x^2 + y^2;
  //  g = (1 + k(1)*rr + k(2)*rr^2);
  //  dp = [2*k(3)*x*y + k(4)*(rr + 2*x^2); 2*k(4)*x*y + k(3)*(rr + 2*y^2)];
  //  pi(:,i) = g * pn(:,i) + dp;
  const double x = p.x(), y = p.y(), xy = x * y, xx = x * x, yy = y * y;
  const double rr = xx + yy;
  const double r4 = rr * rr;
  const double g = 1. + k1_ * rr + k2_ * r4; // scaling factor

  // tangential component
  const double dx = 2. * p1_ * xy + p2_ * (rr + 2. * xx);
  const double dy = 2. * p2_ * xy + p1_ * (rr + 2. * yy);

  // Radial and tangential distortion applied
  const double pnx = g * x + dx;
  const double pny = g * y + dy;

  Matrix2 DK;
  if (H1 || H2) DK << fx_, s_, 0.0, fy_;

  // Derivative for calibration
  if (H1)
    *H1 = D2dcalibration(x, y, xx, yy, xy, rr, r4, pnx, pny, DK);

  // Derivative for points
  if (H2)
    *H2 = D2dintrinsic(x, y, rr, g, k1_, k2_, p1_, p2_, DK);

  // Regular uncalibrate after distortion
  return Point2(fx_ * pnx + s_ * pny + u0_, fy_ * pny + v0_);
}

/* ************************************************************************* */
Point2 Cal3DS2_Base::calibrate(const Point2& pi, const double tol) const {
  // Use the following fixed point iteration to invert the radial distortion.
  // pn_{t+1} = (inv(K)*pi - dp(pn_{t})) / g(pn_{t})

  const Point2 invKPi ((1 / fx_) * (pi.x() - u0_ - (s_ / fy_) * (pi.y() - v0_)),
                       (1 / fy_) * (pi.y() - v0_));

  // initialize by ignoring the distortion at all, might be problematic for pixels around boundary
  Point2 pn = invKPi;

  // iterate until the uncalibrate is close to the actual pixel coordinate
  const int maxIterations = 10;
  int iteration;
  for (iteration = 0; iteration < maxIterations; ++iteration) {
    if (distance2(uncalibrate(pn), pi) <= tol) break;
    const double x = pn.x(), y = pn.y(), xy = x * y, xx = x * x, yy = y * y;
    const double rr = xx + yy;
    const double g = (1 + k1_ * rr + k2_ * rr * rr);
    const double dx = 2 * p1_ * xy + p2_ * (rr + 2 * xx);
    const double dy = 2 * p2_ * xy + p1_ * (rr + 2 * yy);
    pn = (invKPi - Point2(dx, dy)) / g;
  }

  if ( iteration >= maxIterations )
    throw std::runtime_error("Cal3DS2::calibrate fails to converge. need a better initialization");

  return pn;
}

/* ************************************************************************* */
Matrix2 Cal3DS2_Base::D2d_intrinsic(const Point2& p) const {
  const double x = p.x(), y = p.y(), xx = x * x, yy = y * y;
  const double rr = xx + yy;
  const double r4 = rr * rr;
  const double g = (1 + k1_ * rr + k2_ * r4);
  Matrix2 DK;
  DK << fx_, s_, 0.0, fy_;
  return D2dintrinsic(x, y, rr, g, k1_, k2_, p1_, p2_, DK);
}

/* ************************************************************************* */
Matrix29 Cal3DS2_Base::D2d_calibration(const Point2& p) const {
  const double x = p.x(), y = p.y(), xx = x * x, yy = y * y, xy = x * y;
  const double rr = xx + yy;
  const double r4 = rr * rr;
  const double g = (1 + k1_ * rr + k2_ * r4);
  const double dx = 2 * p1_ * xy + p2_ * (rr + 2 * xx);
  const double dy = 2 * p2_ * xy + p1_ * (rr + 2 * yy);
  const double pnx = g * x + dx;
  const double pny = g * y + dy;
  Matrix2 DK;
  DK << fx_, s_, 0.0, fy_;
  return D2dcalibration(x, y, xx, yy, xy, rr, r4, pnx, pny, DK);
}

}
/* ************************************************************************* */


