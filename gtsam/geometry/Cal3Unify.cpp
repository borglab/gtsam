/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file Cal3Unify.cpp
 * @date Mar 8, 2014
 * @author Jing Dong
 */

#include <gtsam/base/Vector.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/Cal3Unify.h>

#include <cmath>

namespace gtsam {

/* ************************************************************************* */
Cal3Unify::Cal3Unify(const Vector &v):
    xi_(v[0]), fx_(v[1]), fy_(v[2]), s_(v[3]), u0_(v[4]), v0_(v[5]), k1_(v[6]), k2_(v[7]), k3_(v[8]), k4_(v[9]){}

/* ************************************************************************* */
Matrix Cal3Unify::K() const {
  return (Matrix(3, 3) << fx_, s_, u0_, 0.0, fy_, v0_, 0.0, 0.0, 1.0);
}

/* ************************************************************************* */
Vector Cal3Unify::vector() const {
  return (Vector(10) << xi_, fx_, fy_, s_, u0_, v0_, k1_, k2_, k3_, k4_);
}

/* ************************************************************************* */
void Cal3Unify::print(const std::string& s) const {
  gtsam::print(K(), s + ".K");
  gtsam::print(Vector(k()), s + ".k");
  gtsam::print(Vector(xi_), s + ".xi");
}

/* ************************************************************************* */
bool Cal3Unify::equals(const Cal3Unify& K, double tol) const {
  if (fabs(fx_ - K.fx_) > tol || fabs(fy_ - K.fy_) > tol || fabs(s_ - K.s_) > tol ||
      fabs(u0_ - K.u0_) > tol || fabs(v0_ - K.v0_) > tol || fabs(k1_ - K.k1_) > tol ||
      fabs(k2_ - K.k2_) > tol || fabs(k3_ - K.k3_) > tol || fabs(k4_ - K.k4_) > tol ||
      fabs(xi_ - K.xi_) > tol)
    return false;
  return true;
}

/* ************************************************************************* */
Point2 Cal3Unify::uncalibrate(const Point2& p,
       boost::optional<Matrix&> H1,
       boost::optional<Matrix&> H2) const {

  // this part of code is modified from Cal3DS2,
  // since the second part of this model (after project to normalized plane)
  // is same as Cal3DS2

  // parameters
  const double xi = xi_, fx = fx_, fy = fy_, s = s_;
  const double k1 = k1_, k2 = k2_, k3 = k3_, k4 = k4_;

  // Part1: project 3D space to NPlane
  const double xs = p.x(), ys = p.y();  // normalized points in 3D space
  const double sqrt_nx = sqrt(xs * xs + ys * ys + 1.0);
  const double xi_sqrt_nx = 1 + xi * sqrt_nx;
  const double xi_sqrt_nx2 = xi_sqrt_nx * xi_sqrt_nx;
  const double x = xs / xi_sqrt_nx, y = ys / xi_sqrt_nx; // points on NPlane

  // Part2: project NPlane point to pixel plane: same as Cal3DS2
  const double xy = x * y, xx = x * x, yy = y * y;
  const double rr = xx + yy;
  const double r4 = rr * rr;
  const double g = 1. + k1 * rr + k2 * r4;
  const double dx = 2. * k3 * xy + k4 * (rr + 2. * xx);
  const double dy = 2. * k4 * xy + k3 * (rr + 2. * yy);

  const double pnx = g*x + dx;
  const double pny = g*y + dy;

  // DDS2 will be used both in H1 and H2
  Matrix DDS2;
  if (H1 || H2) {
    // part2
    const double dr_dx = 2. * x;
    const double dr_dy = 2. * y;
    const double dg_dx = k1 * dr_dx + k2 * 2. * rr * dr_dx;
    const double dg_dy = k1 * dr_dy + k2 * 2. * rr * dr_dy;

    const double dDx_dx = 2. * k3 * y + k4 * (dr_dx + 4. * x);
    const double dDx_dy = 2. * k3 * x + k4 * dr_dy;
    const double dDy_dx = 2. * k4 * y + k3 * dr_dx;
    const double dDy_dy = 2. * k4 * x + k3 * (dr_dy + 4. * y);

    Matrix DK = (Matrix(2, 2) << fx, s_, 0.0, fy);
    Matrix DR = (Matrix(2, 2) << g + x * dg_dx + dDx_dx, x * dg_dy + dDx_dy,
        y * dg_dx + dDy_dx, g + y * dg_dy + dDy_dy);

    DDS2 = DK * DR;
  }

  // Inlined derivative for calibration
  if (H1) {
    // part1
    Matrix DU = (Matrix(2,1) << xs * sqrt_nx / xi_sqrt_nx2,
        ys * sqrt_nx / xi_sqrt_nx2);
    Matrix DDS2U = DDS2 * DU;
    // part2
    Matrix DDS2V = (Matrix(2, 9) <<  pnx, 0.0, pny, 1.0, 0.0, fx * x * rr + s * y * rr,
        fx * x * r4 + s * y * r4, fx * 2. * xy + s * (rr + 2. * yy),
        fx * (rr + 2. * xx) + s * (2. * xy), 0.0, pny, 0.0, 0.0, 1.0,
        fy * y * rr, fy * y * r4, fy * (rr + 2. * yy), fy * (2. * xy));

    *H1 = collect(2, &DDS2U, &DDS2V);
  }
  // Inlined derivative for points
  if (H2) {
    // part1
    Matrix DU = (Matrix(2, 2) << (xi_sqrt_nx - xs * xs / sqrt_nx) / xi_sqrt_nx2,
        -(ys * ys / (sqrt_nx * xi_sqrt_nx2)),
        -(xs * xs / (sqrt_nx * xi_sqrt_nx2)),
        (xi_sqrt_nx - ys * ys / sqrt_nx) / xi_sqrt_nx2);

    *H2 = DDS2 * DU;
  }


  return Point2(fx * pnx + s * pny + u0_, fy * pny + v0_);
}

/* ************************************************************************* */
Point2 Cal3Unify::calibrate(const Point2& pi, const double tol) const {
  // Use the following fixed point iteration to invert the radial distortion.
  // pn_{t+1} = (inv(K)*pi - dp(pn_{t})) / g(pn_{t})

  // point on the normalized plane, input for DS2
  Point2 pnpl = this->imageToNPlane(pi);
  double px = pnpl.x();
  double py = pnpl.y();
  const Point2 invKPi ((1 / fx_) * (px - u0_ - (s_ / fy_) * (py - v0_)),
                       (1 / fy_) * (py - v0_));

  // initialize by ignoring the distortion at all, might be problematic for pixels around boundary
  Point2 pn = invKPi;

  // iterate until the uncalibrate is close to the actual pixel coordinate
  const int maxIterations = 10;
  int iteration;
  for ( iteration = 0; iteration < maxIterations; ++iteration ) {

    if ( uncalibrate(pn).distance(pi) <= tol ) break;

    // part1: image -> normalized plane
    pnpl = this->imageToNPlane(pn);
    // part2: normalized plane -> 3D space
    px = pnpl.x(), py = pnpl.y();
    const double xy = px*py, xx = px*px, yy = py*py;
    const double rr = xx + yy;
    const double g = (1+k1_*rr+k2_*rr*rr);
    const double dx = 2*k3_*xy + k4_*(rr+2*xx);
    const double dy = 2*k4_*xy + k3_*(rr+2*yy);
    pn = (invKPi - Point2(dx,dy))/g;
  }

  if ( iteration >= maxIterations )
    throw std::runtime_error("Cal3DS2::calibrate fails to converge. need a better initialization");

  return pn;
}
/* ************************************************************************* */
Point2 Cal3Unify::imageToNPlane(const Point2& p) const {

  const double x = p.x(), y = p.y();
  const double xy2 = x * x + y * y;
  const double sq_xy = (xi_ + sqrt(1 + (1 - xi_ * xi_) * xy2)) / (xy2 + 1);

  return Point2((sq_xy * x / (sq_xy - xi_)), (sq_xy * y / (sq_xy - xi_)));
}

/* ************************************************************************* */
Cal3Unify Cal3Unify::retract(const Vector& d) const {
  return Cal3Unify(vector() + d);
}

/* ************************************************************************* */
Vector Cal3Unify::localCoordinates(const Cal3Unify& T2) const {
  return T2.vector() - vector();
}

}
/* ************************************************************************* */


