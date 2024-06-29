/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file Cal3DS2_k3_Base.cpp
 * @date Dec 18, 2023
 * @author demul
 * @author Hyeonjin Jeong
 */

#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Cal3DS2_k3_Base.h>
#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/Point3.h>

namespace gtsam {
/* ************************************************************************* */
Vector10 Cal3DS2_k3_Base::vector() const {
  Vector10 v;
  v << fx_, fy_, s_, u0_, v0_, k1_, k2_, p1_, p2_, k3_;
  return v;
}

/* ************************************************************************* */
std::ostream& operator<<(std::ostream& os, const Cal3DS2_k3_Base& cal) {
  os << (Cal3&)cal;
  os << ", k1: " << cal.k1() << ", k2: " << cal.k2() << ", p1: " << cal.p1() << ", p2: " << cal.p2()
     << ", k3: " << cal.k3();
  return os;
}

/* ************************************************************************* */
void Cal3DS2_k3_Base::print(const std::string& s_) const {
  gtsam::print((Matrix)K(), s_ + ".K");
  gtsam::print(Vector(k()), s_ + ".k");
}

/* ************************************************************************* */
bool Cal3DS2_k3_Base::equals(const Cal3DS2_k3_Base& K, double tol) const {
  const Cal3* base = dynamic_cast<const Cal3*>(&K);
  return Cal3::equals(*base, tol) && std::fabs(k1_ - K.k1_) < tol && std::fabs(k2_ - K.k2_) < tol &&
         std::fabs(p1_ - K.p1_) < tol && std::fabs(p2_ - K.p2_) < tol &&
         std::fabs(k3_ - K.k3_) < tol;
}

/* ************************************************************************* */
Eigen::Matrix<double, 2, 10> D2dcalibration(double x,
                                            double y,
                                            double xx,
                                            double yy,
                                            double xy,
                                            double rr,
                                            double r4,
                                            double r6,
                                            double pnx,
                                            double pny,
                                            const Matrix2& DK) {
  Matrix25 DR1;
  DR1 << pnx, 0.0, pny, 1.0, 0.0, 0.0, pny, 0.0, 0.0, 1.0;
  Matrix25 DR2;
  DR2 << x * rr, x * r4, 2 * xy, rr + 2 * xx, x * r6,  //
      y * rr, y * r4, rr + 2 * yy, 2 * xy, y * r6;
  Eigen::Matrix<double, 2, 10> D;
  D << DR1, DK * DR2;
  return D;
}

/* ************************************************************************* */
static Matrix2 D2dintrinsic(double x,
                            double y,
                            double rr,
                            double r4,
                            double g,
                            double k1,
                            double k2,
                            double p1,
                            double p2,
                            double k3,
                            const Matrix2& DK) {
  const double drdx = 2. * x;
  const double drdy = 2. * y;
  const double dgdx = k1 * drdx + k2 * 2. * rr * drdx + k3 * 3 * r4 * drdx;
  const double dgdy = k1 * drdy + k2 * 2. * rr * drdy + k3 * 3 * r4 * drdy;

  // Dx = 2*p1*xy + p2*(rr+2*xx);
  // Dy = 2*p2*xy + p1*(rr+2*yy);
  const double dDxdx = 2. * p1 * y + p2 * (drdx + 4. * x);
  const double dDxdy = 2. * p1 * x + p2 * drdy;
  const double dDydx = 2. * p2 * y + p1 * drdx;
  const double dDydy = 2. * p2 * x + p1 * (drdy + 4. * y);

  Matrix2 DR;
  DR << g + x * dgdx + dDxdx, x * dgdy + dDxdy,  //
      y * dgdx + dDydx, g + y * dgdy + dDydy;

  return DK * DR;
}

/* ************************************************************************* */
Point2 Cal3DS2_k3_Base::uncalibrate(const Point2& p,
                                    OptionalJacobian<2, 10> Dcal,
                                    OptionalJacobian<2, 2> Dp) const {
  //  r² = x² + y²;
  //  g = (1 + k(1)*r² + k(2)*r⁴ + k(5)*r⁶);
  //  dp = [2*k(3)*x*y + k(4)*(r² + 2*x²); 2*k(4)*x*y + k(3)*(r² + 2*y²)];
  //  pi(:,i) = g * pn(:,i) + dp;
  const double x = p.x(), y = p.y(), xy = x * y, xx = x * x, yy = y * y;
  const double rr = xx + yy;
  const double r4 = rr * rr;
  const double r6 = rr * rr * rr;
  const double g = 1. + k1_ * rr + k2_ * r4 + k3_ * r6;  // scaling factor

  // tangential component
  const double dx = 2. * p1_ * xy + p2_ * (rr + 2. * xx);
  const double dy = 2. * p2_ * xy + p1_ * (rr + 2. * yy);

  // Radial and tangential distortion applied
  const double pnx = g * x + dx;
  const double pny = g * y + dy;

  Matrix2 DK;
  if (Dcal || Dp) {
    DK << fx_, s_, 0.0, fy_;
  }

  // Derivative for calibration
  if (Dcal) {
    *Dcal = D2dcalibration(x, y, xx, yy, xy, rr, r4, r6, pnx, pny, DK);
  }

  // Derivative for points
  if (Dp) {
    *Dp = D2dintrinsic(x, y, rr, r4, g, k1_, k2_, p1_, p2_, k3_, DK);
  }

  // Regular uncalibrate after distortion
  return Point2(fx_ * pnx + s_ * pny + u0_, fy_ * pny + v0_);
}

/* ************************************************************************* */
Point2 Cal3DS2_k3_Base::calibrate(const Point2& pi,
                                  OptionalJacobian<2, 10> Dcal,
                                  OptionalJacobian<2, 2> Dp) const {
  // Use the following fixed point iteration to invert the radial distortion.
  // pn_{t+1} = (inv(K)*pi - dp(pn_{t})) / g(pn_{t})

  const Point2 invKPi((1 / fx_) * (pi.x() - u0_ - (s_ / fy_) * (pi.y() - v0_)),
                      (1 / fy_) * (pi.y() - v0_));

  // initialize by ignoring the distortion at all, might be problematic for
  // pixels around boundary
  Point2 pn = invKPi;

  // iterate until the uncalibrate is close to the actual pixel coordinate
  const int maxIterations = 10;
  int iteration;
  for (iteration = 0; iteration < maxIterations; ++iteration) {
    if (distance2(uncalibrate(pn), pi) <= tol_) break;
    const double px = pn.x(), py = pn.y(), xy = px * py, xx = px * px, yy = py * py;
    const double rr = xx + yy;
    const double r4 = rr * rr;
    const double r6 = rr * rr * rr;
    const double g = (1 + k1_ * rr + k2_ * r4 + k3_ * r6);
    const double dx = 2 * p1_ * xy + p2_ * (rr + 2 * xx);
    const double dy = 2 * p2_ * xy + p1_ * (rr + 2 * yy);
    pn = (invKPi - Point2(dx, dy)) / g;
  }

  if (iteration >= maxIterations)
    throw std::runtime_error("Cal3DS2::calibrate fails to converge. need a better initialization");

  calibrateJacobians<Cal3DS2_k3_Base, dimension>(*this, pn, Dcal, Dp);

  return pn;
}

/* ************************************************************************* */
Matrix2 Cal3DS2_k3_Base::D2d_intrinsic(const Point2& p) const {
  const double x = p.x(), y = p.y(), xx = x * x, yy = y * y;
  const double rr = xx + yy;
  const double r4 = rr * rr;
  const double r6 = rr * rr * rr;
  const double g = (1 + k1_ * rr + k2_ * r4 + k3_ * r6);
  Matrix2 DK;
  DK << fx_, s_, 0.0, fy_;
  return D2dintrinsic(x, y, rr, r4, g, k1_, k2_, p1_, p2_, k3_, DK);
}

/* ************************************************************************* */
Eigen::Matrix<double, 2, 10> Cal3DS2_k3_Base::D2d_calibration(const Point2& p) const {
  const double x = p.x(), y = p.y(), xx = x * x, yy = y * y, xy = x * y;
  const double rr = xx + yy;
  const double r4 = rr * rr;
  const double r6 = rr * rr * rr;
  const double g = (1 + k1_ * rr + k2_ * r4 + k3_ * r6);
  const double dx = 2 * p1_ * xy + p2_ * (rr + 2 * xx);
  const double dy = 2 * p2_ * xy + p1_ * (rr + 2 * yy);
  const double pnx = g * x + dx;
  const double pny = g * y + dy;
  Matrix2 DK;
  DK << fx_, s_, 0.0, fy_;
  return D2dcalibration(x, y, xx, yy, xy, rr, r4, r6, pnx, pny, DK);
}
}  // namespace gtsam
/* ************************************************************************* */
