/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file Cal3DS2.cpp
 * @date Feb 28, 2010
 * @author ydjian
 */

#include <gtsam/base/Vector.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Cal3DS2.h>

namespace gtsam {

/* ************************************************************************* */
Cal3DS2::Cal3DS2(const Vector &v):
    fx_(v[0]), fy_(v[1]), s_(v[2]), u0_(v[3]), v0_(v[4]), k1_(v[5]), k2_(v[6]), p1_(v[7]), p2_(v[8]){}

/* ************************************************************************* */
Matrix Cal3DS2::K() const {
  return (Matrix(3, 3) << fx_, s_, u0_, 0.0, fy_, v0_, 0.0, 0.0, 1.0);
}

/* ************************************************************************* */
Vector Cal3DS2::vector() const {
  return (Vector(9) << fx_, fy_, s_, u0_, v0_, k1_, k2_, p1_, p2_);
}

/* ************************************************************************* */
void Cal3DS2::print(const std::string& s_) const {
  gtsam::print(K(), s_ + ".K");
  gtsam::print(Vector(k()), s_ + ".k");
}

/* ************************************************************************* */
bool Cal3DS2::equals(const Cal3DS2& K, double tol) const {
  if (fabs(fx_ - K.fx_) > tol || fabs(fy_ - K.fy_) > tol || fabs(s_ - K.s_) > tol ||
      fabs(u0_ - K.u0_) > tol || fabs(v0_ - K.v0_) > tol || fabs(k1_ - K.k1_) > tol ||
      fabs(k2_ - K.k2_) > tol || fabs(p1_ - K.p1_) > tol || fabs(p2_ - K.p2_) > tol)
    return false;
  return true;
}

/* ************************************************************************* */
static Matrix D2dcalibration(double x, double y, double xx, double yy,
    double xy, double rr, double r4, double fx, double fy, double s, double pnx,
    double pny) {
  return (Matrix(2, 9) << //
      pnx, 0.0, pny, 1.0, 0.0, //
  fx * x * rr + s * y * rr, fx * x * r4 + s * y * r4, //
  fx * 2 * xy + s * (rr + 2 * yy), fx * (rr + 2 * xx) + s * (2 * xy), //
  0.0, pny, 0.0, 0.0, 1.0, //
  fy * y * rr, fy * y * r4, //
  fy * (rr + 2 * yy), fy * (2 * xy));
}

/* ************************************************************************* */
static Matrix D2dintrinsic(double x, double y, double rr, double g, double fx,
    double fy, double s, double k1, double k2, double p1, double p2) {
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

  Matrix DK = (Matrix(2, 2) << fx, s, 0.0, fy);
  Matrix DR = (Matrix(2, 2) << //
      g + x * dgdx + dDxdx, x * dgdy + dDxdy, //
  y * dgdx + dDydx, g + y * dgdy + dDydy);

  return DK * DR;
}

/* ************************************************************************* */
Point2 Cal3DS2::uncalibrate(const Point2& p, boost::optional<Matrix&> H1,
    boost::optional<Matrix&> H2) const {

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

  // Derivative for calibration
  if (H1)
    *H1 = D2dcalibration(x, y, xx, yy, xy, rr, r4, fx_, fy_, s_, pnx, pny);

  // Derivative for points
  if (H2)
    *H2 = D2dintrinsic(x, y, rr, g, fx_, fy_, s_, k1_, k2_, p1_, p2_);

  // Regular uncalibrate after distortion
  return Point2(fx_ * pnx + s_ * pny + u0_, fy_ * pny + v0_);
}

/* ************************************************************************* */
Point2 Cal3DS2::calibrate(const Point2& pi, const double tol) const {
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
    if (uncalibrate(pn).distance(pi) <= tol) break;
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
Matrix Cal3DS2::D2d_intrinsic(const Point2& p) const {
  const double x = p.x(), y = p.y(), xx = x * x, yy = y * y;
  const double rr = xx + yy;
  const double r4 = rr * rr;
  const double g = (1 + k1_ * rr + k2_ * r4);
  return D2dintrinsic(x, y, rr, g, fx_, fy_, s_, k1_, k2_, p1_, p2_);
}

/* ************************************************************************* */
Matrix Cal3DS2::D2d_calibration(const Point2& p) const {
  const double x = p.x(), y = p.y(), xx = x * x, yy = y * y, xy = x * y;
  const double rr = xx + yy;
  const double r4 = rr * rr;
  const double g = (1 + k1_ * rr + k2_ * r4);
  const double dx = 2 * p1_ * xy + p2_ * (rr + 2 * xx);
  const double dy = 2 * p2_ * xy + p1_ * (rr + 2 * yy);
  const double pnx = g * x + dx;
  const double pny = g * y + dy;
  return D2dcalibration(x, y, xx, yy, xy, rr, r4, fx_, fy_, s_, pnx, pny);
}

/* ************************************************************************* */
Cal3DS2 Cal3DS2::retract(const Vector& d) const {
  return Cal3DS2(vector() + d);
}

/* ************************************************************************* */
Vector Cal3DS2::localCoordinates(const Cal3DS2& T2) const {
  return T2.vector() - vector();
}

}
/* ************************************************************************* */


